"""RoboSub front camera: camera, YOLO and the front camera detectors.

Data sources (via `sim` arg):
  sim:=false  — launches RealSense D555 attached to the shared container;
                images published on /{drone}/front_camera/image_color + camera_info
  sim:=true   — skips camera; simulator publishes on the same topics

Inference backend (via `backend` arg):
  none        — no YOLO inference (the detectors get no input)
  ultralytics — YOLO BB on the front image

Detectors (each on/off with its own arg):
  slalom      — slalom_pole_finder: pipe boxes → 3D pipes on the landmarks topic
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from auv_setup.launch_arg_common import (
    declare_drone_and_namespace_args,
    resolve_drone_and_namespace,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

_DETECTIONS_TOPIC = '/yolo/robosub_front_detections'
_ANNOTATED_TOPIC = '/yolo/robosub_front_annotated'


def _launch_setup(context, *args, **kwargs):
    pkg_dir = get_package_share_directory('perception_setup')
    drone, namespace = resolve_drone_and_namespace(context)

    with open(
        os.path.join(
            get_package_share_directory('auv_setup'),
            'config',
            'robots',
            f'{drone}.yaml',
        )
    ) as f:
        robot_topics = yaml.safe_load(f)['/**']['ros__parameters']['topics']

    enable_gstreamer = (
        LaunchConfiguration('enable_gstreamer').perform(context).lower() == 'true'
    )
    use_nvidia = (
        LaunchConfiguration('gst_nvidia_encoder').perform(context).lower() == 'true'
    )
    destination_ip = LaunchConfiguration('destination_ip').perform(context)
    yolo_destination_port = int(
        LaunchConfiguration('yolo_destination_port').perform(context)
    )

    backend = LaunchConfiguration('backend').perform(context)
    model_file_path = LaunchConfiguration('model_file_path').perform(context)
    device = LaunchConfiguration('device').perform(context)
    visualize = LaunchConfiguration('visualize').perform(context)
    confidence_threshold = LaunchConfiguration('confidence_threshold').perform(context)
    enable_slalom = LaunchConfiguration('slalom').perform(context).lower() == 'true'
    sim = LaunchConfiguration('sim').perform(context).lower() == 'true'

    color_image_topic = f'/{namespace}/front_camera/image_color'
    landmarks_topic = (
        LaunchConfiguration('landmarks_topic').perform(context)
        or f'/{namespace}/{robot_topics["landmarks"]}'
    )

    container_nodes = []
    if enable_gstreamer and backend != 'none':
        container_nodes.append(
            ComposableNode(
                package='gstreamer_from_ros',
                plugin='gstreamer_from_ros::GStreamerFromRos',
                name='gstreamer_yolo',
                parameters=[
                    {
                        'input_topic': _ANNOTATED_TOPIC,
                        'destination_ip': destination_ip,
                        'destination_port': yolo_destination_port,
                        'bitrate': 500000,
                        'expected_input_fps': 15,
                        'preset_level': 1,
                        'iframe_interval': 15,
                        'control_rate': 1,
                        'pt': 96,
                        'config_interval': 1,
                        'input_format': 'BGR',
                        'hw_encoder': use_nvidia,
                    }
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            )
        )

    # The RealSense attaches to this container when sim:=false.
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
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        pkg_dir,
                        'launch',
                        'ultralytics',
                        'ultralytics_yolo_bb.launch.py',
                    )
                ),
                launch_arguments={
                    'node_name': 'yolo_robosub_front',
                    'model_input_image_topic': color_image_topic,
                    'model_file_path': model_file_path,
                    'detections_topic': _DETECTIONS_TOPIC,
                    'annotated_image_topic': _ANNOTATED_TOPIC,
                    'confidence_threshold': confidence_threshold,
                    'device': device,
                    'visualize': visualize,
                }.items(),
            )
        )

    if enable_slalom:
        actions.append(
            Node(
                package='slalom_pole_finder',
                executable='slalom_pole_finder_node',
                name='slalom_pole_finder',
                output='screen',
                parameters=[
                    os.path.join(
                        get_package_share_directory('slalom_pole_finder'),
                        'config',
                        'slalom_params.yaml',
                    ),
                    {
                        'detections_topic': _DETECTIONS_TOPIC,
                        'landmarks_topic': landmarks_topic,
                        'camera_frame': f'{namespace}/front_camera_color_optical',
                        'odom_frame': f'{namespace}/odom',
                    },
                    # The simulator's slalom poles (red) are 0.938 m, not 0.9 m.
                    *([{'object_height': 0.938}] if sim else []),
                ],
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
                'backend',
                default_value='ultralytics',
                choices=['none', 'ultralytics'],
                description='YOLO BB inference backend. none = disabled.',
            ),
            DeclareLaunchArgument(
                'model_file_path',
                default_value=os.path.join(pkg_dir, 'models', 'best_slalom.pt'),
                description='Path to the YOLO BB model file.',
            ),
            DeclareLaunchArgument(
                'device',
                default_value='0',
                description="Inference device: 'cpu', GPU index, 'cuda', 'cuda:N', or 'mps'.",
            ),
            DeclareLaunchArgument(
                'visualize',
                default_value='true',
                description='Publish annotated images from YOLO BB.',
            ),
            DeclareLaunchArgument(
                'confidence_threshold',
                default_value='0.50',
                description='YOLO confidence threshold.',
            ),
            DeclareLaunchArgument(
                'slalom',
                default_value='true',
                choices=['true', 'false'],
                description='Run slalom_pole_finder on the YOLO detections.',
            ),
            DeclareLaunchArgument(
                'landmarks_topic',
                default_value='',
                description=(
                    'Where the detectors publish. Empty = the landmarks topic '
                    'in the drone config.'
                ),
            ),
            DeclareLaunchArgument(
                'enable_gstreamer',
                default_value='false',
                description='Stream the YOLO annotated image via GStreamer/RTP.',
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
                'yolo_destination_port',
                default_value='5003',
                description='Destination UDP port for the YOLO GStreamer RTP stream.',
            ),
            DeclareLaunchArgument(
                'resolution',
                default_value='1280x800',
                choices=['896x504', '1280x800'],
                description='RealSense resolution preset.',
            ),
            DeclareLaunchArgument(
                'fps',
                default_value='15',
                description='RealSense camera frame rate.',
            ),
            OpaqueFunction(function=_launch_setup),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('perception_setup'),
                        'launch',
                        'cameras',
                        'realsense_d555.launch.py',
                    )
                ),
                launch_arguments={
                    'drone': LaunchConfiguration('drone'),
                    'resolution': LaunchConfiguration('resolution'),
                    'fps': LaunchConfiguration('fps'),
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
