"""Norbit FLS sonar — full composable pipeline.

All nodes run inside a single component container.  Two modes:

  standalone=true  (default)
    Creates its own ComposableNodeContainer named by `container_name`.

  standalone=false
    Uses LoadComposableNodes to attach the nodes to an already-running
    container identified by `container_name`.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def _launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)
    vres = int(LaunchConfiguration('vres').perform(context))
    hres = int(LaunchConfiguration('hres').perform(context))
    horizontal_fov_deg = float(LaunchConfiguration('horizontal_fov_deg').perform(context))
    range_start = float(LaunchConfiguration('range_start').perform(context))
    range_stop = float(LaunchConfiguration('range_stop').perform(context))
    datatype_uint8 = LaunchConfiguration('datatype_uint8').perform(context).lower() == 'true'
    enable_gstreamer = LaunchConfiguration('enable_gstreamer').perform(context).lower() == 'true'
    gst_nvidia = LaunchConfiguration('gst_nvidia_encoder').perform(context).lower() == 'true'
    destination_ip = LaunchConfiguration('destination_ip').perform(context)
    destination_port = int(LaunchConfiguration('destination_port').perform(context))
    standalone = LaunchConfiguration('standalone').perform(context).lower() == 'true'
    container_name = LaunchConfiguration('container_name').perform(context)

    # Absolute topic path for gstreamer input, accounting for optional namespace
    display_topic = (
        f'/{namespace}/fls_image/display_mono' if namespace else '/fls_image/display_mono'
    )

    nodes = [
        ComposableNode(
            package='norbit_fls_ros_interface',
            plugin='NorbitFLSRosInterface',
            name='norbit_fls_ros_interface_node',
            namespace=namespace,
            parameters=[{
                'raw_sonar_image_pub_topic': 'fls_image/image',
                'cartesian_sonar_image_pub_topic': 'fls_image/display_mono',
                'norbit_sonar_info_pub_topic': 'fls/norbit_sonar_info',
                'sonar_info_pub_topic': 'fls/sonar_info',
                'frame_id': 'fls_frame',
                'connection_params.ip': '192.168.3.7',
                'connection_params.bathy_port': 2210,
                'connection_params.water_column_port': 2211,
                'connection_params.snippet_sidescan_port': 2212,
                'connection_params.cmd_port': 2209,
                'connection_params.cmd_timeout_sec': 2.0,
                'sonar_settings.mode': 'Bathy',
                'sonar_settings.vres': vres,
                'sonar_settings.hres': hres,
                'sonar_settings.horizontal_fov_deg': horizontal_fov_deg,
                'sonar_settings.range_start': range_start,
                'sonar_settings.range_stop': range_stop,
                'sonar_settings.datatype_uint8': datatype_uint8,
            }],
            extra_arguments=[{'use_intra_process_comms': True}],
        ),
    ]

    if enable_gstreamer:
        nodes.append(
            ComposableNode(
                package='image_to_gstreamer',
                plugin='image_to_gstreamer::ImageToGStreamer',
                name='image_to_gstreamer_node',
                parameters=[{
                    'input_topic': display_topic,
                    'destination_ip': destination_ip,
                    'destination_port': destination_port,
                    'bitrate': 500000,
                    'expected_input_fps': 10,
                    'preset_level': 1,
                    'iframe_interval': 10,
                    'control_rate': 1,
                    'pt': 96,
                    'config_interval': 1,
                    'input_format': 'GRAY8',
                    'hw_encoder': gst_nvidia,
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
            'namespace',
            default_value='',
            description='ROS namespace for all sonar nodes and topics',
        ),
        DeclareLaunchArgument(
            'vres',
            default_value='1024',
            description='Sonar vertical resolution in pixels.',
        ),
        DeclareLaunchArgument(
            'hres',
            default_value='256',
            description='Sonar horizontal resolution in pixels.',
        ),
        DeclareLaunchArgument(
            'horizontal_fov_deg',
            default_value='140.0',
            description='Sonar horizontal field of view in degrees.',
        ),
        DeclareLaunchArgument(
            'range_start',
            default_value='0.0',
            description='Sonar range start in metres.',
        ),
        DeclareLaunchArgument(
            'range_stop',
            default_value='30.0',
            description='Sonar range stop in metres.',
        ),
        DeclareLaunchArgument(
            'datatype_uint8',
            default_value='true',
            description='Use uint8 instead of uint16 for the cartesian sonar image.',
            choices=['true', 'false'],
        ),
        DeclareLaunchArgument(
            'enable_gstreamer',
            default_value='false',
            description='Stream the sonar display image via GStreamer/RTP H.265',
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
            default_value='5002',
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
            default_value='sonar_container',
            description='Container name to create (standalone=true) or attach to (standalone=false)',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
