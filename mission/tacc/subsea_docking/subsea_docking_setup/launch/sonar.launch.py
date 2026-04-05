import os

from ament_index_python.packages import get_package_share_directory
from auv_setup.launch_arg_common import (
    declare_drone_and_namespace_args,
    resolve_drone_and_namespace,
)
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):
    drone, namespace = resolve_drone_and_namespace(context)

    sonar_config = os.path.join(
        get_package_share_directory("norbit_fls_ros_interface"),
        "config",
        "norbit_fls_ros_interface_params.yaml",
    )

    container = ComposableNodeContainer(
        name="sonar_container",
        namespace=namespace,
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="norbit_fls_ros_interface",
                plugin="NorbitFLSRosInterface",
                name="norbit_fls_ros_interface_node",
                namespace=namespace,
                parameters=[sonar_config],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
        output="screen",
        arguments=["--ros-args", "--log-level", "error"],
    )

    return [container]


def generate_launch_description():
    return LaunchDescription(
        declare_drone_and_namespace_args() + [OpaqueFunction(function=launch_setup)]
    )
