"""Valve detection pipeline — full entry point.

Backends (via `backend` arg):
  isaac_ros   — TensorRT OBB inference via isaac_ros/isaac_ros_yolo_obb.launch.py
  ultralytics — Python OBB inference via ultralytics/ultralytics_yolo_obb.launch.py

Data sources (via `sim` arg):
  sim:=false  — launches RealSense D555 attached to the mission container; color and depth
                images are remapped to /{drone}/front_camera/ and
                /{drone}/depth_camera/ before downstream nodes see them.
  sim:=true   — skips the camera; the simulator is expected to publish on
                the same /{drone}/front_camera/ and /{drone}/depth_camera/
                topics so all downstream nodes are source-agnostic.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

_DETECTIONS_TOPIC = '/valve_detection/obb_detections'
_ANNOTATED_TOPIC = '/valve_detection/annotated_image'


def _launch_setup(context, *args, **kwargs):
    pkg_dir = get_package_share_directory('perception_setup')

    backend = LaunchConfiguration('backend').perform(context)
    sim = LaunchConfiguration('sim').perform(context).lower() == 'true'
    drone = LaunchConfiguration('drone').perform(context)
    enable_undistort = (
        LaunchConfiguration('enable_undistort').perform(context).lower() == 'true'
    )
    image_width = int(LaunchConfiguration('image_width').perform(context))
    image_height = int(LaunchConfiguration('image_height').perform(context))
    device = LaunchConfiguration('device').perform(context)
    enable_gstreamer = (
        LaunchConfiguration('enable_gstreamer').perform(context).lower() == 'true'
    )
    use_nvidia = (
        LaunchConfiguration('gst_nvidia_encoder').perform(context).lower() == 'true'
    )
    destination_ip = LaunchConfiguration('destination_ip').perform(context)
    destination_port = int(LaunchConfiguration('destination_port').perform(context))
    visualize = LaunchConfiguration('visualize').perform(context)
    enable_subtype_resolver = (
        LaunchConfiguration('enable_subtype_resolver').perform(context).lower()
        == 'true'
    )
    enable_auto_white_balance = LaunchConfiguration(
        'enable_auto_white_balance'
    ).perform(context)
    white_balance = LaunchConfiguration('white_balance').perform(context)

    # All downstream nodes subscribe to these drone-prefixed topics.
    # Real hardware: camera + undistort publish here.
    # Sim: simulator is expected to publish here directly.
    color_image_topic = f'/{drone}/front_camera/image_color'
    color_info_topic = f'/{drone}/front_camera/camera_info'
    depth_image_topic = f'/{drone}/depth_camera/image_depth'
    depth_info_topic = f'/{drone}/depth_camera/camera_info'

    installed_launch_dir = os.path.join(pkg_dir, 'launch')

    # -------------------------------------------------------------------------
    # OBB inference backend — resolved first so ValvePoseNode params are known
    # -------------------------------------------------------------------------
    if backend == 'isaac_ros':
        detections_letterboxed = True
        undistort_detections = False if sim else (not enable_undistort)
        obb_launch_file = os.path.join(
            installed_launch_dir, 'isaac_ros', 'isaac_ros_yolo_obb.launch.py'
        )
        obb_args = {
            'image_topic': color_image_topic,
            'camera_info_topic': color_info_topic,
            'image_width': str(image_width),
            'image_height': str(image_height),
            'detections_topic': _DETECTIONS_TOPIC,
            'annotated_image_topic': _ANNOTATED_TOPIC,
            'visualize': visualize,
            'standalone': 'false',
            'container_name': 'valve_intervention_container',
        }
    elif backend == 'ultralytics':
        detections_letterboxed = False
        undistort_detections = False if sim else (not enable_undistort)
        obb_launch_file = os.path.join(
            installed_launch_dir, 'ultralytics', 'ultralytics_yolo_obb.launch.py'
        )
        obb_args = {
            'model_input_image_topic': color_image_topic,
            'detections_topic': _DETECTIONS_TOPIC,
            'annotated_image_topic': _ANNOTATED_TOPIC,
            'device': device,
            'visualize': visualize,
        }
    else:
        raise RuntimeError(
            f"Unknown backend '{backend}'. Must be 'isaac_ros' or 'ultralytics'."
        )

    actions = []
    container_name = 'valve_intervention_container'

    container_nodes = [
        ComposableNode(
            package='valve_detection',
            plugin='valve_detection::ValvePoseNode',
            name='valve_pose_node',
            parameters=[
                os.path.join(
                    get_package_share_directory('valve_detection'),
                    'config',
                    'valve_detection_params.yaml',
                ),
                {
                    'detections_sub_topic': _DETECTIONS_TOPIC,
                    'depth_image_sub_topic': depth_image_topic,
                    'depth_image_info_topic': depth_info_topic,
                    'color_image_info_topic': color_info_topic,
                    'color_image_sub_topic': color_image_topic,
                    'depth_frame_id': 'front_camera_depth_optical',
                    'color_frame_id': 'front_camera_color_optical',
                    'landmarks_pub_topic': '/valve_landmarks',
                    'output_frame_id': 'front_camera_depth_optical',
                    'drone': drone,
                    'undistort_detections': undistort_detections,
                    'detections_letterboxed': detections_letterboxed,
                    'debug_visualize': LaunchConfiguration('debug_visualize'),
                    'use_hardcoded_extrinsic': LaunchConfiguration(
                        'use_hardcoded_extrinsic'
                    ),
                    'extrinsic_tx': -0.0588846690952778,
                    'extrinsic_ty': 7.41585317882709e-05,
                    'extrinsic_tz': 0.000453426211606711,
                    'extrinsic_R': [
                        0.999998,
                        0.00057367,
                        0.00211211,
                        -0.00057441,
                        1.0,
                        0.00034676,
                        -0.00211191,
                        -0.00034797,
                        0.999998,
                    ],
                },
            ],
        ),
    ]

    if enable_subtype_resolver:
        container_nodes.append(
            ComposableNode(
                package='valve_subtype_resolver',
                plugin='valve_subtype_resolver::ValveSubtypeResolverNode',
                name='valve_subtype_resolver_node',
                parameters=[
                    os.path.join(
                        get_package_share_directory('valve_subtype_resolver'),
                        'config',
                        'valve_subtype_resolver_params.yaml',
                    ),
                    {'drone': drone, 'landmarks_pub_topic': drone + '/landmarks'},
                ],
            )
        )

    if enable_gstreamer:
        container_nodes.append(
            ComposableNode(
                package='gstreamer_from_ros',
                plugin='gstreamer_from_ros::GStreamerFromRos',
                name='gstreamer_from_ros_node',
                extra_arguments=[{'use_intra_process_comms': True}],
                parameters=[
                    {
                        'input_topic': _ANNOTATED_TOPIC,
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
                    }
                ],
            )
        )

    # -------------------------------------------------------------------------
    # Main container — owns ValvePoseNode and GStreamer; camera nodes attach here
    # -------------------------------------------------------------------------
    actions.append(
        ComposableNodeContainer(
            name=container_name,
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=container_nodes,
            output='screen',
            additional_env={'EGL_PLATFORM': 'surfaceless'},
        )
    )

    # -------------------------------------------------------------------------
    # Camera + image processing — attached to the mission container.
    # In sim mode the simulator publishes the final topics directly, so skip.
    # For bag replay (real topic names, no live camera) set enable_camera:=false
    # so only the undistort + crop nodes run against the bagged raw topics.
    # -------------------------------------------------------------------------
    if not sim:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        installed_launch_dir, 'cameras', 'realsense_d555.launch.py'
                    )
                ),
                launch_arguments={
                    'drone': drone,
                    'enable_camera': LaunchConfiguration('enable_camera'),
                    'enable_undistort': LaunchConfiguration('enable_undistort'),
                    'enable_gstreamer': 'false',
                    'standalone': 'false',
                    'container_name': container_name,
                    'enable_auto_white_balance': enable_auto_white_balance,
                    'white_balance': white_balance,
                }.items(),
            )
        )

    # -------------------------------------------------------------------------
    # OBB inference backend (owns its own container)
    # -------------------------------------------------------------------------
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(obb_launch_file),
            launch_arguments=obb_args.items(),
        )
    )

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'backend',
                default_value='isaac_ros',
                choices=['isaac_ros', 'ultralytics'],
                description="OBB inference backend: 'isaac_ros' (TensorRT) or 'ultralytics' (Python)",
            ),
            DeclareLaunchArgument(
                'sim',
                default_value='false',
                description=(
                    'false = launch RealSense D555 attached to the mission container; '
                    'true = skip camera (simulator publishes on /{drone}/front_camera/ topics)'
                ),
            ),
            DeclareLaunchArgument(
                'enable_camera',
                default_value='true',
                description=(
                    'When sim:=false, controls whether the live RealSense camera '
                    'node is launched. Set false for bag replay against raw '
                    '/camera/camera/color/image_raw — undistort + crop still run.'
                ),
            ),
            DeclareLaunchArgument(
                'drone',
                default_value='nautilus',
                description='Robot name, used as topic/TF namespace prefix',
            ),
            DeclareLaunchArgument(
                'enable_undistort',
                default_value='true',
                description='Apply lens undistortion (sim:=false only)',
            ),
            DeclareLaunchArgument(
                'image_width',
                default_value='896',
                description='Input image width (must match camera profile; sim may differ)',
            ),
            DeclareLaunchArgument(
                'image_height',
                default_value='504',
                description='Input image height',
            ),
            DeclareLaunchArgument(
                'device',
                default_value='0',
                description="Ultralytics inference device: 'cpu', GPU index, 'cuda', 'cuda:N', or 'mps'",
            ),
            DeclareLaunchArgument(
                'debug_visualize',
                default_value='true',
                description='Enable valve_detection debug visualisation topics',
            ),
            DeclareLaunchArgument(
                'use_hardcoded_extrinsic',
                default_value='true',
                description='Use hardcoded depth-to-color extrinsic instead of TF lookup',
            ),
            DeclareLaunchArgument(
                'visualize',
                default_value='true',
                description='Launch the OBB visualizer and publish annotated images',
            ),
            DeclareLaunchArgument(
                'enable_subtype_resolver',
                default_value='true',
                description='Launch the valve_subtype_resolver node alongside the pipeline',
            ),
            DeclareLaunchArgument(
                'enable_gstreamer',
                default_value='true',
                description='Stream annotated image over GStreamer/RTP to 10.0.0.169:5000',
            ),
            DeclareLaunchArgument(
                'gst_nvidia_encoder',
                default_value='true',
                description='Use NVIDIA hardware H.265 encoder (nvv4l2h265enc). '
                'Set false to use software x265enc.',
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
                'enable_auto_white_balance',
                default_value='false',
                description='Enable auto white balance on the color camera. When false, white_balance is used.',
            ),
            DeclareLaunchArgument(
                'white_balance',
                default_value='4500.0',
                description='Manual white balance value (Kelvin). Only applied when enable_auto_white_balance=false.',
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
