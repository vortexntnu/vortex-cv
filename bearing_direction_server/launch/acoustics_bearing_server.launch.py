import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("bearing_direction_server"),
        "config", "bearing_direction_server.yaml",
    )

    return LaunchDescription([
        Node(
            package="bearing_direction_server",
            executable="acoustics_bearing_server",
            name="acoustics_bearing_server",
            output="screen",
            parameters=[
                config,
                {
                    "action_name": "acoustics_bearing_direction",
                    "topics.bearing_measurements": "acoustics/bearing_measurements",
                },
            ],
        ),
    ])
