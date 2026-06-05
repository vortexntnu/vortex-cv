"""Pipeline inspection front camera + YOLO segmentation.

Data sources (via `sim` arg):
  sim:=false  — launches RealSense D555 attached to the shared container;
                images published on /{drone}/front_camera/image_color + camera_info
  sim:=true   — skips camera; simulator publishes on the same topics

Inference backend (via `backend` arg):
  none        — no YOLO inference
  ultralytics — run seg on the front image
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from auv_setup.launch_arg_common import (
    declare_drone_and_namespace_args,
    resolve_drone_and_namespace,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

_SEG_DEBUG_IMAGE_TOPIC = '/pipeline/front_camera/segmentation_debug'


def _launch_setup(context, *args, **kwargs):
    pkg_dir = get_package_share_directory('perception_setup')
    drone, namespace = resolve_drone_and_namespace(context)

    with open(os.path.join(
        get_package_share_directory('auv_setup'), 'config', 'robots', f'{drone}.yaml',
    )) as f:
        robot_topics = yaml.safe_load(f)['/**']['ros__parameters']['topics']
    enable_gstreamer = LaunchConfiguration('enable_gstreamer').perform(context).lower() == 'true'
    use_nvidia = LaunchConfiguration('gst_nvidia_encoder').perform(context).lower() == 'true'
    destination_ip = LaunchConfiguration('destination_ip').perform(context)
    destination_port = int(LaunchConfiguration('destination_port').perform(context))

    backend = LaunchConfiguration('backend').perform(context)
    model_file_path = LaunchConfiguration('model_file_path').perform(context)
    device = LaunchConfiguration('device').perform(context)
    visualize = LaunchConfiguration('visualize').perform(context)

    installed_launch_dir = os.path.join(pkg_dir, 'launch')

    color_image_topic = f'/{namespace}/front_camera/image_color'
    camera_info_topic = f'/{namespace}/front_camera/camera_info'

    container_nodes = []

    if enable_gstreamer:
        container_nodes.append(
            ComposableNode(
                package='gstreamer_from_ros',
                plugin='gstreamer_from_ros::GStreamerFromRos',
                name='gstreamer_from_ros_node',
                parameters=[{
                    'input_topic': _SEG_DEBUG_IMAGE_TOPIC,
                    'destination_ip': destination_ip,
                    'destination_port': destination_port,
                    'bitrate': 500000,
                    'expected_input_fps': 15,
                    'preset_level': 1,
                    'iframe_interval': 15,
                    'control_rate': 1,
                    'pt': 96,
                    'config_interval': 1,
                    'input_format': 'RGB',
                    'hw_encoder': use_nvidia,
                }],
                extra_arguments=[{'use_intra_process_comms': True}],
            )
        )

    actions = [
        ComposableNodeContainer(
            name='front_camera_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=container_nodes,
            output='screen',
            additional_env={'EGL_PLATFORM': 'surfaceless'},
        )
    ]

    if backend == 'ultralytics':
        seg_launch_file = os.path.join(
            installed_launch_dir, 'ultralytics', 'ultralytics_yolo_seg.launch.py'
        )
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(seg_launch_file),
                launch_arguments={
                    'node_name': 'yolo_seg_front_camera',
                    'model_input_image_topic': color_image_topic,
                    'camera_info_topic': camera_info_topic,
                    'model_file_path': model_file_path,
                    'output_bbox_topic': '/pipeline/front_camera/bboxes',
                    'output_mask_topic': '/pipeline/front_camera/segmentation_mask',
                    'output_debug_topic': '/pipeline/front_camera/segmentation_debug',
                    'output_camera_info_topic': '/pipeline/front_camera/camera_info',
                    'output_mask_overlay_topic': '/pipeline/front_camera/segmentation_overlay',
                    'device': device,
                    'pub_debug': visualize,
                    'pub_mask_overlay': 'true',
                }.items(),
            )
        )

    actions.append(
        Node(
            package='pipeline_image_endpoints_detector',
            executable='image_endpoints_node',
            name='pipeline_image_endpoints',
            parameters=[{
                'morph_kernel_size': 5,
                'detection_method': 'lowest_pixel',
                'input_topic': '/pipeline/front_camera/segmentation_mask',
                'output_topic': '/pipeline/image_endpoints',
                'debug_topic': '/pipeline/image_endpoints/debug_image',
                'debug': True,
            }],
            arguments=['--ros-args', '--log-level', 'pipeline_image_endpoints:=info'],
            output='screen',
        )
    )

    actions.append(
        Node(
            package='pipeline_endpoint_position_estimator',
            executable='position_estimator_node',
            name='pipeline_position_estimator',
            parameters=[{
                'endpoints_topic': '/pipeline/image_endpoints',
                'dvl_altitude_topic': f'/{namespace}/{robot_topics["dvl_altitude"]}',
                'camera_info_topic': '/pipeline/front_camera/camera_info',
                'publish_topic': f'/{namespace}/{robot_topics["landmarks"]}',
                'transform_timeout_ms': 100,
                'apply_undistortion': True,
                'reference_frame': f'{namespace}/odom',
            }],
            arguments=['--ros-args', '--log-level', 'pipeline_position_estimator:=info'],
            output='screen',
        )
    )

    return actions


def generate_launch_description():
    pkg_dir = get_package_share_directory('perception_setup')

    return LaunchDescription(
        declare_drone_and_namespace_args()
        + [
            DeclareLaunchArgument(
                'sim',
                default_value='false',
                choices=['true', 'false'],
                description=(
                    'false = launch RealSense D555 attached to the shared container; '
                    'true = skip camera (simulator publishes on /{drone}/front_camera/ topics)'
                ),
            ),
            DeclareLaunchArgument(
                'enable_gstreamer',
                default_value='true',
                description='Stream the seg debug image via GStreamer/RTP',
            ),
            DeclareLaunchArgument(
                'gst_nvidia_encoder',
                default_value='true',
                description='Use NVIDIA hardware H.265 encoder. Set false for software x265enc.',
            ),
            DeclareLaunchArgument(
                'destination_ip',
                default_value='10.0.0.169',
                description='Destination IP for GStreamer RTP stream.',
            ),
            DeclareLaunchArgument(
                'destination_port',
                default_value='5000',
                description='Destination UDP port for GStreamer RTP stream.',
            ),
            DeclareLaunchArgument(
                'backend',
                default_value='ultralytics',
                choices=['none', 'ultralytics'],
                description='YOLO inference backend. ultralytics = seg on front image.',
            ),
            DeclareLaunchArgument(
                'model_file_path',
                default_value=os.path.join(pkg_dir, 'models', 'pipe-real-front-05-06-m.pt'),
                description='Path to the YOLO segmentation model file.',
            ),
            DeclareLaunchArgument(
                'device',
                default_value='cuda',
                description="Inference device: 'cpu', GPU index, 'cuda', 'cuda:N', or 'mps'.",
            ),
            DeclareLaunchArgument(
                'visualize',
                default_value='true',
                description='Publish debug annotated images from YOLO seg.',
            ),
            OpaqueFunction(function=_launch_setup),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('perception_setup'),
                        'launch', 'cameras', 'realsense_d555.launch.py',
                    )
                ),
                launch_arguments={
                    'drone': LaunchConfiguration('drone'),
                    'enable_depth': 'false',
                    'enable_undistort': 'true',
                    'enable_gstreamer': 'false',
                    'standalone': 'false',
                    'container_name': 'front_camera_container',
                }.items(),
                condition=UnlessCondition(LaunchConfiguration('sim')),
            ),
        ]
    )
