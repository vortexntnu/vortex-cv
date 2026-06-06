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
                "action_servers.bearing_direction": "acoustics_bearing_direction",
                "action_servers.pipeline_bearing_direction": "pipeline_bearing_direction",
                "bearing_collection_timeout_sec": 10.0,
                "acoustic_bearing_projection_distance": 3.0,
                "pipeline_bearing_projection_distance": 5.0,
                "bearing_waypoint_altitude": 1.0,
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

    pipeline_bearing_server = Node(
        package="bearing_direction_server",
        executable="pipeline_bearing_server",
        name="pipeline_bearing_server",
        namespace=namespace,
        parameters=[
            drone_config,
            {
                "action_name": "pipeline_bearing_direction",
                "topics.pixel_detections": "/pipeline/image_endpoints",
                "topics.camera_info": "/pipeline/front_camera/camera_info",
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

    return [node, pipeline_bearing_server, irls_line_trigger]


def generate_launch_description():
    return LaunchDescription(
        declare_drone_and_namespace_args()
        + [
            DeclareLaunchArgument(
                'start_in_camera_range',
                default_value='false',
                choices=['true', 'false'],
                description=(
                    'Skip acoustic hunt and start directly with pipeline bearing collection. '
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
