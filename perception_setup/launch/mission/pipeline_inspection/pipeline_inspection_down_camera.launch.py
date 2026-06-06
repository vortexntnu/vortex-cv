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

    with open(os.path.join(
        get_package_share_directory('auv_setup'), 'config', 'robots', f'{drone}.yaml',
    )) as f:
        robot_topics = yaml.safe_load(f)['/**']['ros__parameters']['topics']

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
                'pubs.landmarks': f'/{namespace}/{robot_topics["landmarks"]}',
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
        actions.append(
            Node(
                package='pipeline_end_detector',
                executable='pipeline_end_detector_node',
                name='pipeline_end_detector_node',
                namespace=namespace,
                parameters=[{
                    # Consecutive Class 1 detections required before declaring end of pipeline
                    'detection_threshold': 10,
                    # Delay (s) between the start_detection trigger and detection
                    # becoming active; lets the FSM enter pipeline following
                    # immediately while suppressing end detection for a settling
                    # window. 0 = activate now.
                    'activation_delay_sec': 30.0,
                    # Topic published by the end-of-pipeline classifier (std_msgs/UInt8)
                    'topics.detection': '/pipeline_end_classification/turned_off',
                    # FSM service called when the end of pipeline is reached
                    'topics.end_of_pipeline_service': 'pipeline_inspection_fsm/pipeline_finished',
                    # Service the FSM calls to activate detection on this node
                    'topics.start_detection_service': 'pipeline_end_detector/start_detection',
                }],
                output='screen',
            )
        )

    if backend == 'ultralytics':
        seg_launch_file = os.path.join(
            installed_launch_dir, 'ultralytics', 'ultralytics_yolo_seg.launch.py'
        )
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(seg_launch_file),
                launch_arguments={
                    'node_name': 'yolo_seg_down_camera',
                    'model_input_image_topic': color_image_topic,
                    'camera_info_topic': camera_info_topic,
                    'model_file_path': seg_model_file_path,
                    'output_bbox_topic': '/pipeline/down_camera/bboxes',
                    'output_mask_topic': '/pipeline/down_camera/segmentation_mask',
                    'output_debug_topic': '/pipeline/down_camera/segmentation_debug',
                    'output_mask_overlay_topic': '/pipeline/down_camera/segmentation_overlay',
                    'output_camera_info_topic': '/pipeline/down_camera/camera_info',
                    'device': device,
                    'pub_debug': visualize,
                    'pub_mask_overlay': 'true',
                }.items(),
            )
        )

        actions.append(
            Node(
                package='yolo_classify',
                executable='classifier_node',
                name='classifier_node',
                namespace='yolo',
                output='screen',
                parameters=[{
                    'input_topic': '/pipeline/down_camera/segmentation_mask',
                    'model_path': classify_model_file_path,
                    'device': device,
                    'output_class_topic': '/pipeline_end_classification',
                    'imgsz': 640,
                    'verbose': False,
                }],
            )
        )

    actions.append(
        Node(
            package='irls_line_fitter_2x',
            executable='irls_line_node',
            name='irls_line_node',
            parameters=[{
                'input_topic': '/pipeline/down_camera/segmentation_mask',
                'input_topic_info': '/pipeline/down_camera/camera_info',
                'output_topic_img': '/irls_line/image',
                'output_topic_lines': '/irls_line/lines',
                'binary_threshold': 200,
                'min_pixels': 250,
                'max_irls_iters': 13,
                'eps_change': 1.0e-4,
                'loss': 'tukey',
                'huber_delta': 0.3,
                'tukey_c': 2.835,
                'scale_with_mad': True,
                'draw_thickness': 3,
                'draw_b': 0,
                'draw_g': 0,
                'draw_r': 255,
                'publish_original_if_fail': True,
                'clip_to_object': True,
                'clip_max_dist_px': 6.0,
                'min_segment_length_px': 100.0,
                'find_second_line': True,
                'removal_band_px': 120.0,
                'min_pixels_second': 250,
                'draw2_b': 0,
                'draw2_g': 255,
                'draw2_r': 0,
                'draw_intersection': True,
                'intersection_radius': 10,
            }],
            output='screen',
        )
    )

    actions.append(
        Node(
            package='pipeline_follower_sim',
            executable='pipeline_follower_node',
            name='pipeline_follower_node',
            parameters=[{
                'input_topic_lines': '/irls_line/lines',
                'input_topic_info': '/pipeline/down_camera/camera_info',
                'input_topic_pose': f'/{namespace}/{robot_topics["odom"]}',
                'input_topic_altitude': f'/{namespace}/{robot_topics["dvl_altitude"]}',
                'camera_height': 0.5,
                'send_rate_hz': 3.0,
                'camera_placment_x': 0.4,
                'camera_placment_y': -0.158,
                'camera_placment_z': 0.161,
                'debug_waypoint_topic': '/debug/waypoint',
                'debug_service_off_topic': '/debug/send_waypoints_service_off',
                'target_height': 0.8,
                'receive_frame': f'{namespace}/downwards_camera_optical',
                'target_frame': f'{namespace}/odom',
            }],
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
                default_value=os.path.join(pkg_dir, 'models', 'pipeline_end_classification.pt'),
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
