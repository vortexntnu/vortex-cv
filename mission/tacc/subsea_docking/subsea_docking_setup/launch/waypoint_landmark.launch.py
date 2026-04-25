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

    drone_params = os.path.join(
        get_package_share_directory("auv_setup"),
        "config",
        "robots",
        f"{drone}.yaml",
    )

    landmark_config = os.path.join(
        get_package_share_directory("landmark_server"),
        "config",
        "landmark_server_config.yaml",
    )

    waypoint_manager_node = Node(
        package="waypoint_manager",
        executable="waypoint_manager_node",
        name="waypoint_manager_node",
        namespace=namespace,
        parameters=[
            drone_params,
            {
                "debug.waypoint_publish_mode": "timer",
                "debug.waypoint_topic_name": f"/{namespace}/current_waypoint",
            },
        ],
        output="screen",
    )

    landmark_server_node = Node(
        package="landmark_server",
        executable="landmark_server_node",
        name="landmark_server_node",
        namespace=namespace,
        parameters=[
            landmark_config,
            drone_params,
            {
                "use_sim_time": False,
                "target_frame": f"{namespace}/odom",
            },
        ],
        output="screen",
    )

    return [waypoint_manager_node, landmark_server_node]


def generate_launch_description():
    return LaunchDescription(
        declare_drone_and_namespace_args() + [OpaqueFunction(function=launch_setup)]
    )
