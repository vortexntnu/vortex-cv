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
    _, namespace = resolve_drone_and_namespace(context)

    config = os.path.join(
        get_package_share_directory('pipeline_end_detector'),
        'config',
        'pipeline_end_detector_config.yaml',
    )

    node = Node(
        package='pipeline_end_detector',
        executable='pipeline_end_detector_node',
        name='pipeline_end_detector_node',
        namespace=namespace,
        parameters=[config],
        output='screen',
    )

    return [node]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        declare_drone_and_namespace_args() + [OpaqueFunction(function=launch_setup)]
    )
