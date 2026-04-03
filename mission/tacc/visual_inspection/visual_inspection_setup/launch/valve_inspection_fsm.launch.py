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

    landmark_convergence_config = os.path.join(
        get_package_share_directory("visual_inspection_fsm"),
        "config",
        "landmark_convergence.yaml",
    )

    node = Node(
        package="visual_inspection_fsm",
        executable="visual_inspection_fsm",
        namespace=namespace,
        parameters=[
            drone_config,
            {
                "landmark_convergence_config": landmark_convergence_config,
                "landmark_convergence_goal_id": "visual_inspection_convergence",
                "landmark.type": 5,  # LandmarkType::VALVE
                "landmark.subtype": 1,  # LandmarkSubtype::VALVE_VERTICAL
            },
        ],
        output="screen",
    )

    return [node]


def generate_launch_description():
    return LaunchDescription(
        declare_drone_and_namespace_args() + [OpaqueFunction(function=launch_setup)]
    )
