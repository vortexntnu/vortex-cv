import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('pipeline_image_endpoints_detector'),
        'config',
        'image_endpoints_params.yaml',
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'log_level',
                default_value='info',
                description='ROS log level (debug, info, warn, error)',
            ),
            Node(
                package='pipeline_image_endpoints_detector',
                executable='image_endpoints_node',
                name='pipeline_image_endpoints',
                parameters=[config_path],
                arguments=[
                    '--ros-args',
                    '--log-level',
                    ['pipeline_image_endpoints:=', LaunchConfiguration('log_level')],
                ],
                output='screen',
            ),
        ]
    )
