from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    config = os.path.join(
        FindPackageShare("bearing_direction_server").perform(None),
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
