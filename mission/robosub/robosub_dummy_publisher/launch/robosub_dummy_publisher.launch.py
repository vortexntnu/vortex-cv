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
    _drone, namespace = resolve_drone_and_namespace(context)

    default_config = os.path.join(
        get_package_share_directory("robosub_dummy_publisher"),
        "config",
        "robosub_dummy_publisher_params.yaml",
    )

    overrides = {
        "seed": LaunchConfiguration("seed").perform(context),
        "rate": float(LaunchConfiguration("rate").perform(context)),
    }
    tasks = LaunchConfiguration("tasks").perform(context)
    if tasks:
        overrides["tasks"] = [t.strip() for t in tasks.split(",") if t.strip()]

    return [
        Node(
            package="robosub_dummy_publisher",
            executable="robosub_dummy_publisher_node",
            name="robosub_dummy_publisher_node",
            namespace=namespace,
            parameters=[default_config, overrides],
            output="screen",
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        declare_drone_and_namespace_args()
        + [
            DeclareLaunchArgument(
                "seed",
                default_value="",
                description=(
                    "Seed for role-dependent landmarks (gate, bin). Empty means "
                    "a fresh draw each run; pass the seed printed by a previous "
                    "run, or vortex-stonefish-sim's robosub_icon_seed, to "
                    "reproduce/match a specific course layout."
                ),
            ),
            DeclareLaunchArgument(
                "tasks",
                default_value="",
                description=(
                    "Comma-separated course elements to publish dummy landmarks "
                    "for (gate, slalom, torpedo_board, bin). Empty means all."
                ),
            ),
            DeclareLaunchArgument(
                "rate",
                default_value="10.0",
                description="Publish rate in Hz.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
