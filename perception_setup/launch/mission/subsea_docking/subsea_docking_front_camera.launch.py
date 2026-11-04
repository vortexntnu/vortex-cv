"""Subsea docking front camera + ArUco detection.

Data sources (via `sim` arg):
  sim:=false  — launches RealSense D555 attached to the shared container;
                images published on /{drone}/front_camera/image_color + camera_info
  sim:=true   — skips camera; simulator publishes on the same topics

ArUco board config (via `target` arg):
  tac    — TAC competition board (marker_size=0.150); use for sim and TAC event
  vortex — Vortex docking plate (marker_size=0.140); use for real-world pool testing
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

    sim = LaunchConfiguration('sim').perform(context).lower() == 'true'
    target = LaunchConfiguration('target').perform(context)
    enable_gstreamer = (
        LaunchConfiguration('enable_gstreamer').perform(context).lower() == 'true'
    )
    use_nvidia = (
        LaunchConfiguration('gst_nvidia_encoder').perform(context).lower() == 'true'
    )
    destination_ip = LaunchConfiguration('destination_ip').perform(context)
    destination_port = int(LaunchConfiguration('destination_port').perform(context))

    backend = LaunchConfiguration('backend').perform(context)
    device = LaunchConfiguration('device').perform(context)
    visualize = LaunchConfiguration('visualize').perform(context)
    model_file_path = LaunchConfiguration('model_file_path').perform(context)
    waypoint_distance = LaunchConfiguration('waypoint_distance').perform(context)
    resolution = LaunchConfiguration('resolution').perform(context)

    installed_launch_dir = os.path.join(pkg_dir, 'launch')
    aruco_params = _TAC_ARUCO if target == 'tac' else _VORTEX_ARUCO

    color_image_topic = f'/{namespace}/front_camera/image_color'
    camera_info_topic = f'/{namespace}/front_camera/camera_info'
    cropped_image_topic = f'/{namespace}/front_camera/image_color_cropped'
    cropped_camera_info_topic = f'/{namespace}/front_camera/camera_info_cropped'
    img_height = int(resolution.split('x')[1])

    container_nodes = [
        ComposableNode(
            package='aruco_detector',
            plugin='ArucoDetectorNode',
            name='front_aruco_detector',
            namespace=namespace,
            parameters=[{
                'subs.image_topic': color_image_topic,
                'subs.camera_info_topic': camera_info_topic,
                'pubs.aruco_image': '/aruco_detector/image_front',
                'pubs.aruco_poses': '/aruco_detector/markers_front',
                'pubs.board_pose': '/aruco_detector/board_front',
                'pubs.landmarks': f'/{namespace}/{robot_topics["landmarks"]}',
                'logger_service_name': '/toggle_marker_logger',
                'detect_board': True,
                'visualize': True,
                'log_markers': False,
                'publish_detections': True,
                'publish_landmarks': True,
                'aruco.dictionary': 'DICT_ARUCO_ORIGINAL',
                'enu_ned_rotation': True,
                'out_tf_frame': f'{namespace}/front_camera_optical',
                **aruco_params,
            }],
            extra_arguments=[{'use_intra_process_comms': True}],
        ),
        ComposableNode(
            package='vortex_cv_util_nodes',
            plugin='vortex_cv_util_nodes::ImageRoiCrop',
            name='front_camera_bottom_crop',
            parameters=[{
                'image_topic': color_image_topic,
                'camera_info_topic': camera_info_topic,
                'output_image_topic': cropped_image_topic,
                'output_camera_info_topic': cropped_camera_info_topic,
                'enable_crop': True,
                'crop.x_offset': 0,
                'crop.y_offset': 0,
                'crop.width': 0,
                'crop.height': img_height - 150,
            }],
            extra_arguments=[{'use_intra_process_comms': True}],
        ),
    ]

    yolo_destination_port = int(
        LaunchConfiguration('yolo_destination_port').perform(context)
    )

    if enable_gstreamer:
        container_nodes.append(
            ComposableNode(
                package='gstreamer_from_ros',
                plugin='gstreamer_from_ros::GStreamerFromRos',
                name='gstreamer_aruco',
                parameters=[
                    {
                        'input_topic': '/aruco_detector/image_front',
                        'destination_ip': destination_ip,
                        'destination_port': destination_port,
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

    container_name = 'front_camera_container'
    actions = [
        ComposableNodeContainer(
            name=container_name,
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=container_nodes,
            output='screen',
            additional_env={'EGL_PLATFORM': 'surfaceless'},
        )
    ]

    if enable_gstreamer and backend != 'none':
        actions.append(
            Node(
                package='gstreamer_from_ros',
                executable='gstreamer_from_ros_node',
                name='gstreamer_yolo',
                parameters=[
                    {
                        'input_topic': '/yolo/annotated_image',
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
                output='screen',
            )
        )

    # -------------------------------------------------------------------------
    # BB inference backend
    # -------------------------------------------------------------------------
    if backend == 'ultralytics':
        bb_launch_file = os.path.join(
            installed_launch_dir, 'ultralytics', 'ultralytics_yolo_bb.launch.py'
        )
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(bb_launch_file),
                launch_arguments={
                    'model_input_image_topic': cropped_image_topic,
                    'model_file_path': model_file_path,
                    'detections_topic': '/yolo/docking_detections',
                    'annotated_image_topic': '/yolo/annotated_image',
                    'device': device,
                    'visualize': visualize,
                }.items(),
            )
        )
        container_nodes.append(
            ComposableNode(
                package='docking_camera_yolo_direction_waypoint',
                plugin='vortex::docking_camera_yolo_direction_waypoint::DockingCameraYoloDirectionWaypointNode',
                name='docking_camera_yolo_direction_waypoint',
                parameters=[{
                    'detection_sub_topic': '/yolo/docking_detections',
                    'camera_info_sub_topic': cropped_camera_info_topic,
                    'landmarks_pub_topic': f'/{namespace}/{robot_topics["landmarks"]}',
                    'odom_frame': f'{namespace}/odom',
                    'waypoint_distance': float(waypoint_distance),
                }],
                extra_arguments=[{'use_intra_process_comms': True}],
            )
        )

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('bearing_direction_server'),
                    'launch',
                    'platform_bearing_server.launch.py',
                )
            ),
            launch_arguments={'namespace': namespace}.items(),
        )
    )

    return actions


def generate_launch_description():
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
                description='Stream ArUco (port 5000) and YOLO (port 5001) annotated images via GStreamer/RTP.',
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
                description='Destination UDP port for ArUco GStreamer RTP stream.',
            ),
            DeclareLaunchArgument(
                'yolo_destination_port',
                default_value='5003',
                description='Destination UDP port for YOLO GStreamer RTP stream.',
            ),
            DeclareLaunchArgument(
                'backend',
                default_value='ultralytics',
                choices=['none', 'ultralytics'],
                description='YOLO BB inference backend. none = disabled.',
            ),
            DeclareLaunchArgument(
                'model_file_path',
                default_value=os.path.join(
                    get_package_share_directory('perception_setup'),
                    'models',
                    'docking_use_this.pt',
                ),
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
                'waypoint_distance',
                default_value='7.5',
                description='Distance [m] ahead of the camera to place the waypoint along the target yaw.',
            ),
            DeclareLaunchArgument(
                'enable_depth',
                default_value='false',
                description='Enable RealSense depth stream and depth crop node.',
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
                    'enable_depth': LaunchConfiguration('enable_depth'),
                    'enable_undistort': 'true',
                    'enable_gstreamer': 'false',
                    'standalone': 'false',
                    'container_name': 'front_camera_container',
                }.items(),
                condition=UnlessCondition(LaunchConfiguration('sim')),
            ),
        ]
    )
