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
            executable="pipeline_bearing_server",
            name="pipeline_bearing_server",
            output="screen",
            parameters=[
                config,
                {
                    "action_name": "pipeline_bearing_direction",
                    "topics.pixel_detections": "pipeline/pixel_detections",
                    "topics.camera_info": "pipeline/camera_info",
                },
            ],
        ),
    ])
