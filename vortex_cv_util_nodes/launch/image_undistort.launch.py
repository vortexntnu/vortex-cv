import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    pkg_dir = get_package_share_directory('vortex_cv_util_nodes')
    default_config = os.path.join(pkg_dir, 'config', 'image_undistort.yaml')

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'params_file',
                default_value=default_config,
                description='Path to the image_undistort parameter YAML',
            ),
            DeclareLaunchArgument(
                'camera_info_file',
                default_value='',
                description='Absolute path to calibration YAML; overrides the value in params_file',
            ),
            DeclareLaunchArgument(
                'enable_undistort',
                default_value='true',
                description='true = undistort, false = passthrough',
            ),
            ComposableNodeContainer(
                name='image_undistort_container',
                namespace='',
                package='rclcpp_components',
                executable='component_container_mt',
                composable_node_descriptions=[
                    ComposableNode(
                        package='vortex_cv_util_nodes',
                        plugin='vortex_cv_util_nodes::ImageUndistort',
                        name='image_undistort',
                        parameters=[
                            LaunchConfiguration('params_file'),
                            {
                                'camera_info_file': LaunchConfiguration(
                                    'camera_info_file'
                                ),
                                'enable_undistort': LaunchConfiguration(
                                    'enable_undistort'
                                ),
                            },
                        ],
                    ),
                ],
                output='screen',
            ),
        ]
    )
