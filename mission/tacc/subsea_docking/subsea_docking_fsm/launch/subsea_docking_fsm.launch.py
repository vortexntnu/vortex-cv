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
        get_package_share_directory('subsea_docking_fsm'),
        'config',
        'fsm_waypoint_config.yaml',
    )

    landmark_convergence_config = os.path.join(
        get_package_share_directory('subsea_docking_fsm'),
        'config',
        'landmark_convergence.yaml',
    )

    drone_config = os.path.join(
        get_package_share_directory('auv_setup'),
        'config',
        'robots',
        f'{drone}.yaml',
    )

    node = Node(
        package='subsea_docking_fsm',
        executable='subsea_docking_fsm',
        namespace=namespace,
        parameters=[
            drone_config,
            {
                'fsm_waypoint_config': fsm_waypoint_config,
                'landmark_convergence_config': landmark_convergence_config,
                'skip_search': False,
                'use_service_waypoint': True,
                'service_request_timeout_sec': 20.0,
                'docking_position_service': 'subsea_docking_fsm/send_docking_position',
                'fallback_waypoint_id': 'fallback_docking_waypoint',
                'landmark_convergence_goal_id': 'power_puck_landmark_convergence',
            },
        ],
        output='screen',
    )

    return [node]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        declare_drone_and_namespace_args() + [OpaqueFunction(function=launch_setup)]
    )
