import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('line_detection_sonar'),
        'config',
        'line_detection_sonar.yaml',
    )

    return LaunchDescription(
        [
            Node(
                package='line_detection_sonar',
                executable='line_detection_sonar_node',
                name='line_detection_sonar',
                output='screen',
                parameters=[
                    config,
                    # {'use_sim_time': True} # If testing with rosbags sim_time might be preferred if bag is looped
                ],
            ),
        ]
    )
