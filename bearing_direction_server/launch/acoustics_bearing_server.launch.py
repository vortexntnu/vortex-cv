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
                "frame_override",
                default_value="",
                description="Override the TF source frame. Empty = use frame_id from message.",
            ),
            Node(
                package="bearing_direction_server",
                executable="acoustics_bearing_server",
                name="acoustics_bearing_server",
                output="screen",
                parameters=[
                    config,
                    {
                        "action_name": "acoustics_bearing_direction",
                        "topics.bearing_measurements": "/nautilus/acoustics/bearing_measurement",
                        "frame_override": LaunchConfiguration("frame_override"),
                    },
                ],
            ),
        ]
    )
