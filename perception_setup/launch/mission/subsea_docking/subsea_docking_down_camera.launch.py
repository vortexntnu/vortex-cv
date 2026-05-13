"""Subsea docking down camera + ArUco detection.

Data sources (via `sim` arg):
  sim:=false  — launches FLIR Blackfly S attached to the shared container;
                image published on /{drone}/down_camera/image_color + camera_info
  sim:=true   — skips camera; simulator publishes on the same topics

ArUco board config (via `target` arg):
  tac    — TAC competition board (marker_size=0.150); use for sim and TAC event
  vortex — Vortex docking plate (marker_size=0.140); use for real-world pool testing
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

    aruco_params = _TAC_ARUCO if target == 'tac' else _VORTEX_ARUCO

    container_nodes = [
        ComposableNode(
            package='aruco_detector',
            plugin='ArucoDetectorNode',
            name='down_aruco_detector',
            namespace=namespace,
            parameters=[{
                'subs.image_topic': f'/{namespace}/down_camera/image_color',
                'subs.camera_info_topic': f'/{namespace}/down_camera/camera_info',
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
                package='image_to_gstreamer',
                plugin='image_to_gstreamer::ImageToGStreamer',
                name='image_to_gstreamer_node',
                parameters=[{
                    'input_topic': _ARUCO_IMAGE_TOPIC,
                    'destination_ip': '10.0.0.68',
                    'destination_port': 5001,
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

    return [
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


def generate_launch_description():
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
                description='Stream the ArUco annotated down image via GStreamer/RTP to 10.0.0.68:5001',
            ),
            DeclareLaunchArgument(
                'gst_nvidia_encoder',
                default_value='true',
                description='Use NVIDIA hardware H.265 encoder. Set false for software x265enc.',
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
