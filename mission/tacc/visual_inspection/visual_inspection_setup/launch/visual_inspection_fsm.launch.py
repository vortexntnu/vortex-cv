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
                "standoff_waypoint_goal_id": "standoff_waypoint",
                "tcp_offset_goal_id": "tcp_offset",
                "vertical_mounted_valve": True,
                "tcp_base_frame": f"{namespace}/base_link",
                "tcp_tip_frame": f"{namespace}/grip_point",
                # Frame of the depth camera used for height-alignment.
                # Must match the TF frame published by the camera driver.
                "depth_camera_frame": f"{namespace}/depth_camera",
                "action_servers.gripper": f"/{namespace}/gripper/reference_filter",
                "gripper_convergence_threshold": 0.0005,
                # Direction to twist the valve handle.
                # "ccw" = counter-clockwise from the drone's perspective when
                # facing the valve. "cw" = clockwise.
                "valve_turn_direction": "ccw",
            },
        ],
        output="screen",
    )

    return [node]


def generate_launch_description():
    return LaunchDescription(
        declare_drone_and_namespace_args() + [OpaqueFunction(function=launch_setup)]
    )
