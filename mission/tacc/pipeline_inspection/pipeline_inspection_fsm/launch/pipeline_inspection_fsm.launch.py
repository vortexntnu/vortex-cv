import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pipeline_inspection_fsm_config = os.path.join(
        get_package_share_directory('pipeline_inspection_fsm'),
        'config',
        'pipeline_inspection_fsm_config.yaml',
    )

    pipeline_inspection_fsm_node = Node(
        package='pipeline_inspection_fsm',
        executable='pipeline_inspection_fsm',
        namespace='orca',
        parameters=[pipeline_inspection_fsm_config],
        output='screen',
    )

    return LaunchDescription([pipeline_inspection_fsm_node])
