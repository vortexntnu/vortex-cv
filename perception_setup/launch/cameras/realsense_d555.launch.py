"""RealSense D555 camera — full composable pipeline.

All nodes run inside a single component container.  Two modes:

  standalone=true  (default)
    Creates its own ComposableNodeContainer named by `container_name`.

  standalone=false
    Uses LoadComposableNodes to attach the nodes to an already-running
    container identified by `container_name`.  Use this when a parent
    launch file owns the container and wants camera nodes in the same process.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode

RESOLUTION_PRESETS = {
    '896x504': {'width': 896, 'height': 504},
    '1280x800': {'width': 1280, 'height': 800},
}


def _launch_setup(context, *args, **kwargs):
    pkg_dir = get_package_share_directory('perception_setup')
    drone = LaunchConfiguration('drone').perform(context)
    enable_undistort = (
        LaunchConfiguration('enable_undistort').perform(context).lower() == 'true'
    )
    enable_gstreamer = (
        LaunchConfiguration('enable_gstreamer').perform(context).lower() == 'true'
    )
    use_nvidia = (
        LaunchConfiguration('gst_nvidia_encoder').perform(context).lower() == 'true'
    )
    destination_ip = LaunchConfiguration('destination_ip').perform(context)
    destination_port = int(LaunchConfiguration('destination_port').perform(context))
    standalone = LaunchConfiguration('standalone').perform(context).lower() == 'true'
    container_name = LaunchConfiguration('container_name').perform(context)
    enable_camera = (
        LaunchConfiguration('enable_camera').perform(context).lower() == 'true'
    )
    enable_depth = (
        LaunchConfiguration('enable_depth').perform(context).lower() == 'true'
    )
    enable_depth_crop = (
        LaunchConfiguration('enable_depth_crop').perform(context).lower() == 'true'
    )
    resolution = LaunchConfiguration('resolution').perform(context)
    fps = LaunchConfiguration('fps').perform(context)
    pixel_format = LaunchConfiguration('pixel_format').perform(context)

    calib_files = {
        '896x504': 'color_realsense_d555_calib_downscale.yaml',
        '1280x800': 'color_realsense_d555_calib.yaml',
    }
    calib_file = os.path.join(pkg_dir, 'config', 'cameras', calib_files[resolution])

    preset = RESOLUTION_PRESETS[resolution]
    color_profile = f'{preset["width"]},{preset["height"]},{fps}'
    # Manually determined to crop to a centered region (tested on the resolution 896x504)
    crop_x_offset = 260
    crop_y_offset = 190
    crop_width = 485
    crop_height = 245

    depth_image_topic = f'/{drone}/depth_camera/image_depth'
    depth_info_topic = f'/{drone}/depth_camera/camera_info'
    depth_image_cropped_topic = f'/{drone}/depth_camera/image_depth_cropped'
    depth_info_cropped_topic = f'/{drone}/depth_camera/camera_info_cropped'

    # When the live camera is off (bag replay), the depth_image_crop node
    # consumes the bag's raw RealSense topics directly and republishes on the
    # namespaced topics that downstream nodes subscribe to — effectively
    # replacing what the camera node would have produced, with cropping applied.
    if enable_camera:
        crop_input_image = depth_image_topic
        crop_input_info = depth_info_topic
        crop_output_image = depth_image_cropped_topic
        crop_output_info = depth_info_cropped_topic
    else:
        crop_input_image = '/camera/camera/depth/image_rect_raw'
        crop_input_info = '/camera/camera/depth/camera_info'
        crop_output_image = depth_image_topic
        crop_output_info = depth_info_topic
    color_image_topic = f'/{drone}/front_camera/image_color'
    color_info_topic = f'/{drone}/front_camera/camera_info'
    color_frame = f'{drone}/front_camera_color_optical'
    depth_frame = f'{drone}/front_camera_depth_optical'

    nodes = []
    if enable_camera:
        nodes.append(
            ComposableNode(
                package='realsense2_camera',
                plugin='realsense2_camera::RealSenseNodeFactory',
                name='camera',
                namespace='camera',
                parameters=[
                    {
                        'enable_color': True,
                        'rgb_camera.color_profile': color_profile,
                        'rgb_camera.color_format': pixel_format,
                        'rgb_camera.enable_auto_white_balance': False,
                        'rgb_camera.white_balance': 4500.0,
                        'rgb_camera.enable_auto_exposure': True,
                        'enable_depth': enable_depth,
                        'depth_module.depth_profile': '896,504,15',
                        'depth_module.depth_format': 'Z16',
                        'depth_module.enable_auto_exposure': True,
                        'depth_module.emitter_enabled': False,
                        'enable_infra1': False,
                        'enable_infra2': False,
                        'enable_gyro': False,
                        'enable_accel': False,
                        'enable_motion': False,
                        'publish_tf': False,
                        'enable_sync': False,
                    }
                ],
                remappings=[
                    ('/camera/camera/depth/image_rect_raw', depth_image_topic),
                    ('/camera/camera/depth/camera_info', depth_info_topic),
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            )
        )
    nodes.append(
        ComposableNode(
            package='vortex_cv_util_nodes',
            plugin='vortex_cv_util_nodes::ImageUndistort',
            name='color_image_undistort',
            parameters=[
                {
                    'image_topic': '/camera/camera/color/image_raw',
                    'camera_info_file': calib_file,
                    'raw_camera_info_topic': '/camera/camera/color/camera_info',
                    'output_image_topic': color_image_topic,
                    'output_camera_info_topic': color_info_topic,
                    'enable_undistort': enable_undistort,
                    'image_qos': 'reliable',
                    'output_frame': color_frame,
                }
            ],
            extra_arguments=[{'use_intra_process_comms': True}],
        )
    )

    if enable_depth:
        nodes.append(
            ComposableNode(
                package='vortex_cv_util_nodes',
                plugin='vortex_cv_util_nodes::ImageRoiCrop',
                name='depth_image_crop',
                parameters=[
                    {
                        'image_topic': crop_input_image,
                        'camera_info_topic': crop_input_info,
                        'output_image_topic': crop_output_image,
                        'output_camera_info_topic': crop_output_info,
                        'crop.x_offset': crop_x_offset,
                        'crop.y_offset': crop_y_offset,
                        'crop.width': crop_width,
                        'crop.height': crop_height,
                        'enable_crop': enable_depth_crop,
                        'output_frame': depth_frame,
                    }
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            )
        )

    if enable_gstreamer:
        nodes.append(
            ComposableNode(
                package='gstreamer_from_ros',
                plugin='gstreamer_from_ros::GStreamerFromRos',
                name='gstreamer_from_ros_node',
                parameters=[
                    {
                        'input_topic': color_image_topic,
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
                extra_arguments=[{'use_intra_process_comms': True}],
            )
        )

    if standalone:
        return [
            ComposableNodeContainer(
                name=container_name,
                namespace='',
                package='rclcpp_components',
                executable='component_container_mt',
                composable_node_descriptions=nodes,
                output='screen',
                arguments=['--ros-args', '--log-level', 'info'],
                additional_env={'EGL_PLATFORM': 'surfaceless'},
            )
        ]
    else:
        return [
            LoadComposableNodes(
                target_container=container_name,
                composable_node_descriptions=nodes,
            )
        ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'drone',
                default_value='nautilus',
                description='Robot name, used as topic/TF namespace prefix',
            ),
            DeclareLaunchArgument(
                'resolution',
                default_value='896x504',
                description='Resolution preset: "896x504" or "1280x800".',
                choices=['896x504', '1280x800'],
            ),
            DeclareLaunchArgument(
                'fps',
                default_value='15',
                description='Camera frame rate (e.g. "15" or "30").',
            ),
            DeclareLaunchArgument(
                'pixel_format',
                default_value='RGB8',
                description='Color pixel format.',
                choices=['RGB8', 'BGR8', 'BGRA8'],
            ),
            DeclareLaunchArgument(
                'enable_camera',
                default_value='true',
                description=(
                    'Launch the RealSense camera node. Set false when replaying a '
                    'bag that already publishes /camera/camera/color/image_raw so '
                    'only the undistort + crop nodes run against the bag.'
                ),
            ),
            DeclareLaunchArgument(
                'enable_undistort',
                default_value='true',
                description='Apply lens undistortion to the color image',
            ),
            DeclareLaunchArgument(
                'enable_gstreamer',
                default_value='false',
                description='Stream the undistorted color image via GStreamer/RTP H.265',
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
                'standalone',
                default_value='true',
                description=(
                    'true = create a new ComposableNodeContainer named by container_name; '
                    'false = attach nodes to an existing container named by container_name'
                ),
            ),
            DeclareLaunchArgument(
                'container_name',
                default_value='realsense_d555_container',
                description='Container name to create (standalone=true) or attach to (standalone=false)',
            ),
            DeclareLaunchArgument(
                'enable_depth',
                default_value='true',
                description='Enable depth stream and depth crop node.',
            ),
            DeclareLaunchArgument(
                'enable_depth_crop',
                default_value='true',
                description='Crop the depth image to a centered ROI. Has no effect when enable_depth=false.',
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
