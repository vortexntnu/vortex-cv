import os

from ament_index_python.packages import get_package_share_directory
from auv_setup.launch_arg_common import (
    declare_drone_and_namespace_args,
    resolve_drone_and_namespace,
)
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    drone, namespace = resolve_drone_and_namespace(context)

    drone_config = os.path.join(
        get_package_share_directory('auv_setup'),
        'config',
        'robots',
        f'{drone}.yaml',
    )

    node = Node(
        package='valve_inspection_fsm',
        executable='valve_inspection_fsm',
        namespace=namespace,
        parameters=[
            drone_config,
            {
                'frames.gripper_frame': 'gripper_tip',
                'frames.base_frame': 'odom',
            },
        ],
        output='screen',
    )

    return [node]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        declare_drone_and_namespace_args() + [OpaqueFunction(function=launch_setup)]
    )
