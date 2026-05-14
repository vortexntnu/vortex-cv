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

    drone_config = os.path.join(
        get_package_share_directory("auv_setup"),
        "config",
        "robots",
        f"{drone}.yaml",
    )

    start_in_range     = LaunchConfiguration('start_in_range').perform(context).lower() == 'true'
    use_camera_direction = LaunchConfiguration('use_camera_direction').perform(context).lower() == 'true'
    use_wall_detection = LaunchConfiguration('use_wall_detection').perform(context).lower() == 'true'
    use_service_waypoint = LaunchConfiguration('use_service_waypoint').perform(context).lower() == 'true'

    node = Node(
        package="subsea_docking_fsm",
        executable="subsea_docking_fsm",
        namespace=namespace,
        parameters=[
            drone_config,
            {
                "fsm_waypoint_config": fsm_waypoint_config,
                "landmark_convergence_config": landmark_convergence_config,
                "start_in_range": start_in_range,
                "use_camera_direction": use_camera_direction,
                "use_wall_detection": use_wall_detection,
                "use_service_waypoint": use_service_waypoint,
                "wall_detection_estimate_timeout_sec": 20.0,
                "camera_direction_timeout_sec": 30.0,
                "wait_before_fallback_sec": 5.0,
                "docking_position_service": "/docking_position_estimator/docking_pose",
                "docking_estimator_start_service": "docking_position_estimator/start_mission",
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
                default_value='true',

                description=(
                    'Skip all estimation and poll immediately for ArUco. '
                    'Use when already close to the docking platform.'
                ),
            ),
            DeclareLaunchArgument(
                'use_camera_direction',
                default_value='false',

                description=(
                    'Enable YOLO camera-direction estimation. Polls for a '
                    'projected waypoint from the front camera and navigates '
                    'toward it before falling back to wall detection or dock config.'
                ),
            ),
            DeclareLaunchArgument(
                'use_wall_detection',
                default_value='false',

                description=(
                    'Enable sonar wall-detection estimation. Waits for a pose '
                    'from the docking position estimator service and navigates '
                    'toward it before falling back to dock config.'
                ),
            ),
            DeclareLaunchArgument(
                'use_service_waypoint',
                default_value='false',

                description=(
                    'Skip all estimation and navigate directly to the fixed '
                    'dock config waypoint defined in fsm_waypoint_config.yaml.'
                ),
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
