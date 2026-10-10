import os

from ament_index_python.packages import get_package_share_directory
from auv_setup.launch_arg_common import (
    declare_drone_and_namespace_args,
    resolve_drone_and_namespace,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    drone, namespace = resolve_drone_and_namespace(context)

    drone_config = os.path.join(
        get_package_share_directory("auv_setup"),
        "config",
        "robots",
        f"{drone}.yaml",
    )

    config_name = LaunchConfiguration("config").perform(context)
    mission_config = os.path.join(
        get_package_share_directory("perception_setup"),
        "config",
        "mission",
        "robosub",
        "mission.yaml" if config_name == "pool" else f"mission_{config_name}.yaml",
    )

    tree_file = os.path.join(
        get_package_share_directory("robosub_mission"),
        "trees",
        "root.xml",
    )

    node = Node(
        package="robosub_mission",
        executable="robosub_mission",
        namespace=namespace,
        parameters=[
            drone_config,
            {
                "tree_file": tree_file,
                "mission_config": mission_config,
                "tick_rate_hz": 10.0,
                "main_tree": LaunchConfiguration("main_tree").perform(context),
            },
        ],
        output="screen",
    )
    return [node]


def generate_launch_description():
    return LaunchDescription(
        declare_drone_and_namespace_args()
        + [
            DeclareLaunchArgument(
                "config",
                default_value="pool",
                description="pool (mission.yaml) or sim (mission_sim.yaml)",
            ),
            DeclareLaunchArgument(
                "main_tree",
                default_value="Main",
                description="Main (the course) or one task, e.g. TestGate",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
