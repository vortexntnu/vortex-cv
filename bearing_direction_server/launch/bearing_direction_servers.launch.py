from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    pkg_share = FindPackageShare("bearing_direction_server").perform(None)
    config = os.path.join(pkg_share, "config", "bearing_direction_server.yaml")

    pipeline_node = Node(
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
    )

    platform_node = Node(
        package="bearing_direction_server",
        executable="platform_bearing_server",
        name="platform_bearing_server",
        output="screen",
        parameters=[
            config,
            {
                "action_name": "platform_bearing_direction",
                "topics.detections": "platform/detections",
                "topics.camera_info": "platform/camera_info",
            },
        ],
    )

    acoustics_node = Node(
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
    )

    return LaunchDescription([pipeline_node, platform_node, acoustics_node])
