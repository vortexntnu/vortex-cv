"""Norbit FLS sonar — full composable pipeline.

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


def _launch_setup(context, *args, **kwargs):
    pkg_dir = get_package_share_directory('norbit_fls_ros_interface')
    config = os.path.join(pkg_dir, 'config', 'norbit_fls_ros_interface_params.yaml')

    namespace = LaunchConfiguration('namespace').perform(context)
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
            parameters=[config],
            extra_arguments=[{'use_intra_process_comms': True}],
        ),
        ComposableNode(
            package='norbit_fls_ros_interface',
            plugin='norbit::ros_interface::SonarOverlayNode',
            name='sonar_overlay_node',
            namespace=namespace,
            parameters=[config],
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
                    'expected_input_fps': 15,
                    'preset_level': 1,
                    'iframe_interval': 15,
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
