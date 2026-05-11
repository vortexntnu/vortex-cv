import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    pkg_dir = get_package_share_directory('subsea_docking_setup')

    line_detection_config = os.path.join(
        pkg_dir, 'config', 'line_detection_ransac.yaml'
    )

    container = ComposableNodeContainer(
        name='subsea_docking_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='line_detection_ransac',
                plugin='LineDetectionRansacNode',
                name='line_detection_ransac',
                parameters=[
                    line_detection_config,
                    {'use_sim_time': False},
                ],
            ),
        ],
        output='screen',
    )

    return LaunchDescription([container])
