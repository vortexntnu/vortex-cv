from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="orca",
        description="ROS namespace for the node.",
    )

    config_arg = DeclareLaunchArgument(
        "config",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("bearing_localization"),
                "config",
                "bearing_localization_config.yaml",
            ]
        ),
        description="Absolute path to a bearing_localization YAML config.",
    )

    profile_arg = DeclareLaunchArgument(
        "profile",
        default_value="aruco",
        description="Parameter profile to use. Options: default, aruco.",
    )

    node = Node(
        package="bearing_localization",
        executable="bearing_localization_node",
        name="bearing_localization_node",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        parameters=[
            LaunchConfiguration("config"),
            {"profile": LaunchConfiguration("profile")},
        ],
    )

    return LaunchDescription([namespace_arg, config_arg, profile_arg, node])
