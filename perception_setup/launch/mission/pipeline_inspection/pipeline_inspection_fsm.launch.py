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
    start_in_camera_range = LaunchConfiguration('start_in_camera_range').perform(context).lower() == 'true'
    start_above_pipe = LaunchConfiguration('start_above_pipe').perform(context).lower() == 'true'

    fsm_waypoint_config = os.path.join(
        get_package_share_directory("perception_setup"),
        "config",
        "mission",
        "pipeline_inspection",
        "search_waypoints.yaml",
    )

    pipeline_convergence_config = os.path.join(
        get_package_share_directory("perception_setup"),
        "config",
        "mission",
        "pipeline_inspection",
        "pipeline_convergence.yaml",
    )

    altitude_descent_waypoint_config = os.path.join(
        get_package_share_directory("perception_setup"),
        "config",
        "mission",
        "pipeline_inspection",
        "altitude_descent_waypoint.yaml",
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
                "altitude_descent_waypoint_config": altitude_descent_waypoint_config,
                "action_servers.bearing_direction": "acoustics_bearing_direction",
                "bearing_collection_timeout_sec": 10.0,
                "bearing_projection_distance": 10.0,
                "services.start_pipeline_following": "pipeline_inspection_fsm/start_pipeline_following",
                "services.start_end_pipeline_detection": "pipeline_end_detector/start_detection",
                "services.end_of_pipeline": "pipeline_inspection_fsm/pipeline_finished",
                "services.irls_line_detected": "/pipeline_inspection_fsm/irls_line_detected",
                "start_in_camera_range": start_in_camera_range,
                "start_above_pipe": start_above_pipe,
            },
        ],
        output="screen",
    )

    irls_line_trigger = Node(
        package="irls_line_trigger",
        executable="irls_line_trigger_node",
        namespace=namespace,
        parameters=[
            {
                "input_topic": "/irls_line/lines",
                "service_name": "/pipeline_inspection_fsm/irls_line_detected",
                "min_call_interval_sec": 0.5,
            },
        ],
        output="screen",
    )

    return [node, irls_line_trigger]


def generate_launch_description():
    return LaunchDescription(
        declare_drone_and_namespace_args()
        + [
            DeclareLaunchArgument(
                'start_in_camera_range',
                default_value='false',
                choices=['true', 'false'],
                description=(
                    'Skip the search phase and start directly in landmark polling. '
                    'Use when the vehicle is already within camera range of the pipeline.'
                ),
            ),
            DeclareLaunchArgument(
                'start_above_pipe',
                default_value='false',
                choices=['true', 'false'],
                description=(
                    'Skip search and convergence entirely and go straight to pipeline following. '
                    'Use when the vehicle is already positioned directly above the pipeline.'
                ),
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
