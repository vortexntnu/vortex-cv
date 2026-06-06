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
                "start_in_range":      arg('start_in_range').lower() == 'true',
                "use_camera_direction": arg('use_camera_direction').lower() == 'true',
                "use_wall_detection":  arg('use_wall_detection').lower() == 'true',
                "use_service_waypoint": arg('use_service_waypoint').lower() == 'true',
                "docking_position_service": "/docking_position_estimator/docking_pose",
                "docking_estimator_start_service": "docking_position_estimator/start_mission",
                # Timing
                "camera_direction_timeout_sec":       float(arg('camera_direction_timeout_sec')),
                "wall_detection_estimate_timeout_sec": float(arg('wall_detection_estimate_timeout_sec')),
                "wait_before_fallback_sec":           float(arg('wait_before_fallback_sec')),
                # Bearing direction server
                "bearing_direction_action_server": arg('bearing_direction_action_server'),
                "bearing_direction_distance":      float(arg('bearing_direction_distance')),
                "bearing_direction_altitude":      float(arg('bearing_direction_altitude')),
                "bearing_direction_min_measurements": int(arg('bearing_direction_min_measurements')),
                "bearing_direction_max_measurements": int(arg('bearing_direction_max_measurements')),
            },
        ],
        output="screen",
    )

    return [node]


def generate_launch_description():
    return LaunchDescription(
        declare_drone_and_namespace_args()
        + [
            DeclareLaunchArgument('start_in_range',       default_value='false',
                description='Skip all estimation and poll immediately for ArUco.'),
            DeclareLaunchArgument('use_camera_direction', default_value='true',
                description='Enable YOLO camera-direction estimation via bearing direction server.'),
            DeclareLaunchArgument('use_wall_detection',   default_value='false',
                description='Enable sonar wall-detection estimation.'),
            DeclareLaunchArgument('use_service_waypoint', default_value='false',
                description='Skip estimation and go directly to fixed dock config waypoint.'),

            # Timing
            DeclareLaunchArgument('camera_direction_timeout_sec',        default_value='30.0',
                description='Bearing collection window (s).'),
            DeclareLaunchArgument('wall_detection_estimate_timeout_sec', default_value='20.0',
                description='Wall detection service timeout (s).'),
            DeclareLaunchArgument('wait_before_fallback_sec',            default_value='5.0',
                description='Wait at estimate waypoint before falling back to dock config (s).'),

            # Bearing direction server
            DeclareLaunchArgument('bearing_direction_action_server', default_value='platform_bearing_direction',
                description='Action server name for the bearing direction collector.'),
            DeclareLaunchArgument('bearing_direction_distance',      default_value='20.0',
                description='Metres to project the waypoint along the averaged bearing.'),
            DeclareLaunchArgument('bearing_direction_altitude',      default_value='1.5',
                description='Absolute z to set on the bearing waypoint (odom frame). Negative = use bearing server z as-is.'),
            DeclareLaunchArgument('bearing_direction_min_measurements', default_value='10',
                description='Minimum YOLO detections required; falls back if not met at timeout.'),
            DeclareLaunchArgument('bearing_direction_max_measurements', default_value='30',
                description='Early-exit: return as soon as this many detections collected (0 = wait full timeout).'),

            OpaqueFunction(function=launch_setup),
        ]
    )
