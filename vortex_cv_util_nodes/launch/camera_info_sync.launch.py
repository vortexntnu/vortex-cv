import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    pkg_dir = get_package_share_directory('vortex_cv_util_nodes')
    default_config = os.path.join(pkg_dir, 'config', 'camera_info_sync.yaml')

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'params_file',
                default_value=default_config,
                description='Path to the camera_info_sync parameter YAML',
            ),
            DeclareLaunchArgument(
                'camera_info_file',
                default_value='',
                description='Absolute path to calibration YAML; overrides the value in params_file',
            ),
            ComposableNodeContainer(
                name='camera_info_sync_container',
                namespace='',
                package='rclcpp_components',
                executable='component_container_mt',
                composable_node_descriptions=[
                    ComposableNode(
                        package='vortex_cv_util_nodes',
                        plugin='vortex_cv_util_nodes::CameraInfoSync',
                        name='camera_info_sync',
                        parameters=[
                            LaunchConfiguration('params_file'),
                            {
                                'camera_info_file': LaunchConfiguration(
                                    'camera_info_file'
                                )
                            },
                        ],
                    ),
                ],
                output='screen',
            ),
        ]
    )
