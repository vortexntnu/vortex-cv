from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('line_detection_ransac')

    config_arg = LaunchConfiguration('config')

    config = PythonExpression(
        ['"', pkg_share, '/config/line_detection_ransac_', config_arg, '.yaml"']
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'config',
                default_value='real',
                choices=['sim', 'real'],
                description='Configuration file to use (sim or real)',
            ),
            Node(
                package='line_detection_ransac',
                executable='line_detection_ransac_node',
                name='line_detection_ransac',
                output='screen',
                parameters=[
                    config,
                    {
                        'use_sim_time': False
                    },  # If testing with rosbags sim_time might be preferred if bag is looped
                ],
            ),
        ]
    )
