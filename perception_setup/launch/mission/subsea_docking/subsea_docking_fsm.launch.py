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

    fsm_waypoint_config = os.path.join(
        get_package_share_directory("perception_setup"),
        "config",
        "mission",
        "subsea_docking",
        "fsm_waypoint_config.yaml",
    )

    landmark_convergence_config = os.path.join(
        get_package_share_directory("perception_setup"),
        "config",
        "mission",
        "subsea_docking",
        "landmark_convergence.yaml",
    )

    def arg(name):
        return LaunchConfiguration(name).perform(context)

    node = Node(
        package="subsea_docking_fsm",
        executable="subsea_docking_fsm",
        namespace=namespace,
        parameters=[
            drone_config,
            {
                "fsm_waypoint_config": fsm_waypoint_config,
                "landmark_convergence_config": landmark_convergence_config,

                # ── Feature flags (also overridable via CLI args) ─────────────
                "start_in_range":      arg('start_in_range').lower() == 'true',
                "use_camera_direction": arg('use_camera_direction').lower() == 'true',
                "use_wall_detection":  arg('use_wall_detection').lower() == 'true',
                "use_service_waypoint": arg('use_service_waypoint').lower() == 'true',

                # ── Services ──────────────────────────────────────────────────
                "docking_position_service": "/docking_position_estimator/docking_pose",
                "docking_estimator_start_service": "docking_position_estimator/start_mission",

                # ── Timing ────────────────────────────────────────────────────
                "camera_direction_timeout_sec": 30.0,
                "wall_detection_estimate_timeout_sec": 20.0,
                "wait_before_fallback_sec": 5.0,

                # ── Bearing direction server ──────────────────────────────────
                "bearing_direction_action_server": "platform_bearing_direction",
                "bearing_direction_distance": 20.0,      # metres along averaged bearing
                "bearing_direction_altitude": 1.1,       # absolute z in odom frame
                "bearing_direction_min_measurements": 10,
                "bearing_direction_max_measurements": 30,
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
                'start_in_range',
                default_value='false',
                choices=['true', 'false'],
                description='Skip all estimation and poll immediately for ArUco.',
            ),
            DeclareLaunchArgument(
                'use_camera_direction',
                default_value='true',
                choices=['true', 'false'],
                description='Enable YOLO camera-direction estimation via bearing direction server.',
            ),
            DeclareLaunchArgument(
                'use_wall_detection',
                default_value='false',
                choices=['true', 'false'],
                description='Enable sonar wall-detection estimation.',
            ),
            DeclareLaunchArgument(
                'use_service_waypoint',
                default_value='false',
                choices=['true', 'false'],
                description='Skip estimation and go directly to fixed dock config waypoint.',
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
