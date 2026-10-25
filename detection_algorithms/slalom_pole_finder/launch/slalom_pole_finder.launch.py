import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('slalom_pole_finder'),
        'config',
        'slalom_params.yaml',
    )

    return LaunchDescription([
        Node(
            package='slalom_pole_finder',
            executable='slalom_pole_finder_node',
            name='slalom_pole_finder',
            output='screen',
            parameters=[config],
        ),
    ])
