import os

import yaml
from ament_index_python.packages import get_package_share_directory
from auv_setup.launch_arg_common import (
    declare_drone_and_namespace_args,
    resolve_drone_and_namespace,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def load_scenario(scenario, namespace):
    path = os.path.join(
        get_package_share_directory("pipeline_inspection_setup"),
        "config",
        "scenarios",
        f"{scenario}.yaml",
    )
    with open(path) as f:
        raw = f.read().replace("{ns}", namespace)
    return yaml.safe_load(raw)["scenario"]


def launch_setup(context, *args, **kwargs):
    drone, namespace = resolve_drone_and_namespace(context)
    scenario = context.launch_configurations["scenario"]
    cfg = load_scenario(scenario, namespace)

    robot_config = os.path.join(
        get_package_share_directory("auv_setup"),
        "config",
        "robots",
        f"{drone}.yaml",
    )

    bearing_share = FindPackageShare("bearing_localization").perform(context)

    bearing_config = os.path.join(
        bearing_share,
        "config",
        "bearing_localization_config.yaml",
    )

    profile = cfg["bearing_localization"]["profile"]
    profile_config = os.path.join(
        bearing_share,
        "config",
        f"{profile}.yaml",
    )

    node = Node(
        package="bearing_localization",
        executable="bearing_localization_node",
        name="bearing_localization_node",
        namespace=namespace,
        output="screen",
        parameters=[
            robot_config,
            bearing_config,
            {"config_file": profile_config},
        ],
    )

    return [node]


def generate_launch_description():
    return LaunchDescription(
        declare_drone_and_namespace_args()
        + [
            DeclareLaunchArgument(
                "scenario",
                default_value="sim",
                description="Scenario to load: sim, real_world",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
