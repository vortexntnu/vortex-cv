import os

from auv_setup.launch_arg_common import (
    declare_drone_and_namespace_args,
    resolve_drone_and_namespace,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    drone, namespace = resolve_drone_and_namespace(context)

    robot_config = os.path.join(
        FindPackageShare("auv_setup").perform(context),
        "config",
        "robots",
        f"{drone}.yaml",
    )

    bearing_config = os.path.join(
        FindPackageShare("bearing_localization").perform(context),
        "config",
        "bearing_localization_config.yaml",
    )

    profile = LaunchConfiguration("profile").perform(context)

    node = Node(
        package="bearing_localization",
        executable="bearing_localization_node",
        name="bearing_localization_node",
        namespace=namespace,
        output="screen",
        parameters=[
            robot_config,
            bearing_config,
            {"profile": profile},
        ],
    )

    return [node]


def generate_launch_description():
    profile_arg = DeclareLaunchArgument(
        "profile",
        default_value="aruco",
        description="Parameter profile to use. Options: default, aruco.",
    )

    return LaunchDescription(
        [
            *declare_drone_and_namespace_args(),
            profile_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )
