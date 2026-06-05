from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="bearing_direction_server",
            executable="test_acoustics_direction_node",
            name="test_acoustics_direction_node",
            output="screen",
            parameters=[{
                # Pipeline start position (world/odom frame)
                "target.x": 1.777425535555405,
                "target.y": -0.053997583427242916,
                "target.z": 5.301166999942316,
                # Angular noise std — tune to match real acoustic sensor
                "noise_std_deg": 3.0,
                "topics.odom": "/nautilus/odom",
                "topics.bearing_measurements": "acoustics/bearing_measurements",
                "publish_rate_hz": 10.0,
            }],
        ),
    ])
