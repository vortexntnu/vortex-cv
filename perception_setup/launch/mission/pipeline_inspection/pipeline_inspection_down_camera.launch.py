"""Pipeline inspection down camera + ArUco detection.

Data sources (via `sim` arg):
  sim:=false  — launches FLIR Blackfly S attached to the shared container;
                image published on /{drone}/down_camera/image_color + camera_info
  sim:=true   — skips camera; simulator publishes on the same topics

ArUco board config (via `target` arg):
  tac    — TAC competition board (marker_size=0.150); use for sim and TAC event
  vortex — Vortex docking plate (marker_size=0.140); use for real-world pool testing

Inference backend (via `backend` arg):
  none        — no YOLO inference
  ultralytics — run seg on the down image, then classify on the seg mask
"""

import os

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
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

_TAC_ARUCO = {
    'aruco.marker_size': 0.150,
    'board.xDist': 0.430,
    'board.yDist': 0.830,
    'board.ids': [28, 7, 96, 19],
}

_VORTEX_ARUCO = {
    'aruco.marker_size': 0.140,
    'board.xDist': 0.448,
    'board.yDist': 0.853,
    'board.ids': [28, 7, 96, 19],
}

_ARUCO_IMAGE_TOPIC = '/aruco_detector/image_down'


def _launch_setup(context, *args, **kwargs):
    pkg_dir = get_package_share_directory('perception_setup')
    drone, namespace = resolve_drone_and_namespace(context)
    target = LaunchConfiguration('target').perform(context)
    enable_gstreamer = LaunchConfiguration('enable_gstreamer').perform(context).lower() == 'true'
    use_nvidia = LaunchConfiguration('gst_nvidia_encoder').perform(context).lower() == 'true'
    destination_ip = LaunchConfiguration('destination_ip').perform(context)
    destination_port = int(LaunchConfiguration('destination_port').perform(context))

    backend = LaunchConfiguration('backend').perform(context)
    seg_model_file_path = LaunchConfiguration('seg_model_file_path').perform(context)
    classify_model_file_path = LaunchConfiguration('classify_model_file_path').perform(context)
    device = LaunchConfiguration('device').perform(context)
    visualize = LaunchConfiguration('visualize').perform(context)

    installed_launch_dir = os.path.join(pkg_dir, 'launch')

    color_image_topic = f'/{namespace}/down_camera/image_color'
    camera_info_topic = f'/{namespace}/down_camera/camera_info'

    aruco_params = _TAC_ARUCO if target == 'tac' else _VORTEX_ARUCO

    container_nodes = [
        ComposableNode(
            package='aruco_detector',
            plugin='ArucoDetectorNode',
            name='down_aruco_detector',
            namespace=namespace,
            parameters=[{
                'subs.image_topic': color_image_topic,
                'subs.camera_info_topic': camera_info_topic,
                'pubs.aruco_image': _ARUCO_IMAGE_TOPIC,
                'pubs.aruco_poses': '/aruco_detector/markers_down',
                'pubs.board_pose': '/aruco_detector/board_down',
                'pubs.landmarks': f'/{namespace}/landmarks',
                'logger_service_name': '/toggle_marker_logger',
                'detect_board': True,
                'visualize': True,
                'log_markers': False,
                'publish_detections': True,
                'publish_landmarks': True,
                'aruco.dictionary': 'DICT_ARUCO_ORIGINAL',
                'enu_ned_rotation': True,
                'out_tf_frame': f'{namespace}/downwards_camera_optical',
                **aruco_params,
            }],
            extra_arguments=[{'use_intra_process_comms': True}],
        ),
    ]

    if enable_gstreamer:
        container_nodes.append(
            ComposableNode(
                package='gstreamer_from_ros',
                plugin='gstreamer_from_ros::GStreamerFromRos',
                name='gstreamer_from_ros_node',
                parameters=[{
                    'input_topic': _ARUCO_IMAGE_TOPIC,
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
            name='down_camera_container',
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
                    'model_input_image_topic': color_image_topic,
                    'camera_info_topic': camera_info_topic,
                    'model_file_path': seg_model_file_path,
                    'output_bbox_topic': '/pipeline/down_camera/bboxes',
                    'output_mask_topic': '/pipeline/down_camera/segmentation_mask',
                    'output_debug_topic': '/pipeline/down_camera/segmentation_debug',
                    'output_camera_info_topic': '/pipeline/down_camera/camera_info',
                    'device': device,
                    'pub_debug': visualize,
                }.items(),
            )
        )

        classify_launch_file = os.path.join(
            installed_launch_dir, 'ultralytics', 'ultralytics_yolo_classify.launch.py'
        )
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(classify_launch_file),
                launch_arguments={
                    'model_input_image_topic': '/pipeline/down_camera/segmentation_mask',
                    'model_file_path': classify_model_file_path,
                    'device': device,
                }.items(),
            )
        )

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('irls_line_fitter_2x'),
                    'launch', 'irls_line.launch.py',
                )
            )
        )
    )

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('pipeline_follower_sim'),
                    'launch', 'pipeline_follower.launch.py',
                )
            )
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
                    'false = launch FLIR Blackfly S attached to the shared container; '
                    'true = skip camera (simulator publishes on /{drone}/down_camera/ topics)'
                ),
            ),
            DeclareLaunchArgument(
                'target',
                default_value='tac',
                choices=['tac', 'vortex'],
                description=(
                    'tac = TAC competition board (marker_size=0.150, xDist=0.430); '
                    'vortex = Vortex docking plate (marker_size=0.140, xDist=0.448)'
                ),
            ),
            DeclareLaunchArgument(
                'enable_gstreamer',
                default_value='true',
                description='Stream the ArUco annotated down image via GStreamer/RTP',
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
                default_value='5001',
                description='Destination UDP port for GStreamer RTP stream.',
            ),
            DeclareLaunchArgument(
                'backend',
                default_value='ultralytics',
                choices=['none', 'ultralytics'],
                description='YOLO inference backend. ultralytics = seg + classify on down image.',
            ),
            DeclareLaunchArgument(
                'seg_model_file_path',
                default_value=os.path.join(pkg_dir, 'models', 'seg_down_with_aruco.pt'),
                description='Path to the YOLO segmentation model file.',
            ),
            DeclareLaunchArgument(
                'classify_model_file_path',
                default_value=os.path.join(pkg_dir, 'models', 'classify_end_of_pipeline.pt'),
                description='Path to the YOLO classification model file.',
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
                        'launch', 'cameras', 'blackfly_s.launch.py',
                    )
                ),
                launch_arguments={
                    'drone': LaunchConfiguration('drone'),
                    'standalone': 'false',
                    'container_name': 'down_camera_container',
                    'enable_gstreamer': 'false',
                }.items(),
                condition=UnlessCondition(LaunchConfiguration('sim')),
            ),
        ]
    )
