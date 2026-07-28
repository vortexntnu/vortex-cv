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
        get_package_share_directory("auv_setup"),
        "config",
        "robots",
        f"{drone}.yaml",
    )

    waypoint_config = os.path.join(
        get_package_share_directory("learning_period"),
        "config",
        "waypoints.yaml",
    )

    node = Node(
        package="learning_period",
        executable="learning_period_fsm",
        namespace=namespace,
        parameters=[
            drone_config,
            {
                "waypoint_config": waypoint_config,
                "action_servers.waypoint_manager": f"/{namespace}/waypoint_manager",
            },
        ],
        output="screen",
    )

    return [node]


def generate_launch_description():
    return LaunchDescription(
        declare_drone_and_namespace_args() + [OpaqueFunction(function=launch_setup)]
    )
