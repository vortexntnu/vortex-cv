"""FLIR Blackfly S downwards camera — full composable pipeline.

All nodes run inside a single component container.  Two modes:

  standalone=true  (default)
    Creates its own ComposableNodeContainer named by `container_name`.

  standalone=false
    Uses LoadComposableNodes to attach the nodes to an already-running
    container identified by `container_name`.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode


RESOLUTION_PRESETS = {
    '720x540':   {'image_width': 720,  'image_height': 540,  'binning_x': 2, 'binning_y': 2, 'calib': 'blackfly_s_calib_downscale.yaml'},
    '1440x1080': {'image_width': 1440, 'image_height': 1080, 'binning_x': 1, 'binning_y': 1, 'calib': 'blackfly_s_calib.yaml'},
}


def _launch_setup(context, *args, **kwargs):
    pkg_dir = get_package_share_directory('perception_setup')

    drone = LaunchConfiguration('drone').perform(context)
    enable_gstreamer = LaunchConfiguration('enable_gstreamer').perform(context).lower() == 'true'
    use_nvidia = LaunchConfiguration('gst_nvidia_encoder').perform(context).lower() == 'true'
    destination_ip = LaunchConfiguration('destination_ip').perform(context)
    destination_port = int(LaunchConfiguration('destination_port').perform(context))
    standalone = LaunchConfiguration('standalone').perform(context).lower() == 'true'
    container_name = LaunchConfiguration('container_name').perform(context)
    resolution = LaunchConfiguration('resolution').perform(context)
    fps = LaunchConfiguration('fps').perform(context)
    pixel_format = LaunchConfiguration('pixel_format').perform(context)

    blackfly_ros_params = os.path.join(pkg_dir, 'config', 'cameras', 'blackfly_s_ros_params.yaml')
    spinnaker_map = os.path.join(pkg_dir, 'config', 'cameras', 'blackfly_s_params.yaml')

    calib_file = RESOLUTION_PRESETS[resolution]['calib'] if resolution in RESOLUTION_PRESETS else 'blackfly_s_calib.yaml'
    calib_path = os.path.join(pkg_dir, 'config', 'cameras', calib_file)

    camera_overrides = {}
    if resolution in RESOLUTION_PRESETS:
        preset = RESOLUTION_PRESETS[resolution]
        camera_overrides.update({
            'image_width':  preset['image_width'],
            'image_height': preset['image_height'],
            'binning_x':    preset['binning_x'],
            'binning_y':    preset['binning_y'],
        })
    if fps:
        camera_overrides['frame_rate'] = float(fps)
    if pixel_format:
        camera_overrides['pixel_format'] = pixel_format

    down_image_topic = f'/{drone}/down_camera/image_color'
    down_info_topic = f'/{drone}/down_camera/camera_info'

    nodes = []

    nodes.append(
        ComposableNode(
            package='spinnaker_camera_driver',
            plugin='spinnaker_camera_driver::CameraDriver',
            name='blackfly_s',
            parameters=[
                blackfly_ros_params,
                {
                    'parameter_file': spinnaker_map,
                    'serial_number': '23494258',
                    'camerainfo_url': f'file://{calib_path}',
                    **camera_overrides,
                },
            ],
            remappings=[
                ('~/control', '/exposure_control/control'),
                ('/blackfly_s/image_raw', down_image_topic),
                ('/blackfly_s/camera_info', down_info_topic),
            ],
            extra_arguments=[{'use_intra_process_comms': True}],
        )
    )

    if enable_gstreamer:
        nodes.append(
            ComposableNode(
                package='image_to_gstreamer',
                plugin='image_to_gstreamer::ImageToGStreamer',
                name='image_to_gstreamer_node',
                parameters=[{
                    'input_topic': down_image_topic,
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

    if standalone:
        return [
            ComposableNodeContainer(
                name=container_name,
                namespace='',
                package='rclcpp_components',
                executable='component_container_mt',
                composable_node_descriptions=nodes,
                output='screen',
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
    return LaunchDescription([
        DeclareLaunchArgument(
            'drone',
            default_value='nautilus',
            description='Robot name, used as topic/TF namespace prefix',
        ),
        DeclareLaunchArgument(
            'resolution',
            default_value='',
            description='Resolution preset: "720x540" (binning 2x2, downscale calib) or "1440x1080" (binning 1x1, full calib). Empty = use YAML defaults.',
            choices=['', '720x540', '1440x1080']
        ),
        DeclareLaunchArgument(
            'fps',
            default_value='',
            description='Override camera frame rate (e.g. "30.0"). Empty = use YAML default.',
        ),
        DeclareLaunchArgument(
            'pixel_format',
            default_value='',
            description='Override pixel format. Empty = use YAML default.',
            choices=['', 'BayerRG8', 'BGR8']
        ),
        DeclareLaunchArgument(
            'enable_gstreamer',
            default_value='false',
            description='Stream the camera image via GStreamer/RTP H.265',
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
            'standalone',
            default_value='true',
            description=(
                'true = create a new ComposableNodeContainer named by container_name; '
                'false = attach nodes to an existing container named by container_name'
            ),
        ),
        DeclareLaunchArgument(
            'container_name',
            default_value='blackfly_s_container',
            description='Container name to create (standalone=true) or attach to (standalone=false)',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
