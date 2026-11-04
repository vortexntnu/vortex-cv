"""Simulator helpers for viewing the map in Foxglove.

Starts identity transforms between world_ned, odom and nautilus/odom,
sim_odom_relay_node and detections_markers_node. Do not use on the vehicle.
"""

from auv_setup.launch_arg_common import (
    declare_drone_and_namespace_args,
    resolve_drone_and_namespace,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    drone, namespace = resolve_drone_and_namespace(context)
    map_frame = f"{drone}/odom"
    sim_frames = IfCondition(LaunchConfiguration("sim_frames"))

    return [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="world_ned_to_map",
            arguments=["--frame-id", map_frame, "--child-frame-id", "world_ned"],
            condition=sim_frames,
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="odom_to_map",
            arguments=["--frame-id", map_frame, "--child-frame-id", "odom"],
            condition=sim_frames,
        ),
        Node(
            package="robosub_dummy_publisher",
            executable="sim_odom_relay_node",
            name="sim_odom_relay_node",
            namespace=namespace,
            parameters=[
                {
                    "odom_frame": f"{drone}/odom",
                    "base_frame": f"{drone}/base_link",
                }
            ],
            output="screen",
            condition=sim_frames,
        ),
        Node(
            package="robosub_dummy_publisher",
            executable="detections_markers_node",
            name="detections_markers_node",
            namespace=namespace,
            output="screen",
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        declare_drone_and_namespace_args()
        + [
            DeclareLaunchArgument(
                "sim_frames",
                default_value="true",
                description=(
                    "Publish the identity transforms from <drone>/odom to "
                    "world_ned and odom (simulator only)."
                ),
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
