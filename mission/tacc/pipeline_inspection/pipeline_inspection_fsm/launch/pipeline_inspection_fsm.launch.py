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

    fsm_waypoint_config = os.path.join(
        get_package_share_directory("pipeline_inspection_fsm"),
        "config",
        "search_waypoints.yaml",
    )

    pipeline_convergence_config = os.path.join(
        get_package_share_directory("pipeline_inspection_fsm"),
        "config",
        "pipeline_convergence.yaml",
    )

    drone_config = os.path.join(
        get_package_share_directory("auv_setup"),
        "config",
        "robots",
        f"{drone}.yaml",
    )

    node = Node(
        package="pipeline_inspection_fsm",
        executable="pipeline_inspection_fsm",
        namespace=namespace,
        parameters=[
            drone_config,
            {
                "fsm_waypoint_config": fsm_waypoint_config,
                "pipeline_convergence_config": pipeline_convergence_config,
                "services.start_pipeline_following": "pipeline_inspection_fsm/start_pipeline_following",
                "services.end_of_pipeline": "pipeline_inspection_fsm/pipeline_finished",
            },
        ],
        output="screen",
    )

    return [node]


def generate_launch_description():
    return LaunchDescription(
        declare_drone_and_namespace_args() + [OpaqueFunction(function=launch_setup)]
    )
