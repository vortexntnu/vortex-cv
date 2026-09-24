"""Helpers for viewing the landmark chain in Foxglove/RViz in the simulator.

- Two static transforms: the simulator publishes odometry in `world_ned` and
  the reference filter its goals in `odom`, while the map is in
  `nautilus/odom`. In the simulator all three are the same axes (odometry
  starts at the world origin), and no node publishes transforms between them,
  so identity transforms let one 3D panel show the vehicle, its goal and the
  map together. Do not use on the real vehicle.
- detections_markers_node: the raw detections on `landmarks` as small markers
  that live 0.3 s, next to the steady map from landmark_server.
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
