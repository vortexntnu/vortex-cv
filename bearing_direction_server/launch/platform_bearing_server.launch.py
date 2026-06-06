from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("bearing_direction_server"),
        "config", "bearing_direction_server.yaml",
    )

    return LaunchDescription([
        Node(
            package="bearing_direction_server",
            executable="platform_bearing_server",
            name="platform_bearing_server",
            namespace="nautilus",
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
    ])
