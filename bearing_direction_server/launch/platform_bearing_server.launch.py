import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("bearing_direction_server"),
        "config",
        "bearing_direction_server.yaml",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'namespace',
                default_value='nautilus',
                description='ROS namespace for the platform bearing direction server.',
            ),
            Node(
                package="bearing_direction_server",
                executable="platform_bearing_server",
                name="platform_bearing_server",
                namespace=LaunchConfiguration('namespace'),
                output="screen",
                parameters=[
                    config,
                    {
                        "action_name": "platform_bearing_direction",
                        "topics.detections": "/yolo/docking_detections",
                        "topics.camera_info": "front_camera/camera_info",
                    },
                ],
            ),
        ]
    )
