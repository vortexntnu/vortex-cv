"""Helpers for viewing the landmark chain in Foxglove/RViz in the simulator.

- detections_markers_node: the raw detections on `landmarks` as small markers
  that live 0.3 s, next to the steady map from landmark_server.

No extra transforms: the frames are the simulator's own (`nautilus/odom`,
`nautilus/odom_enu`, `nautilus/base_link`, ...).
"""

from auv_setup.launch_arg_common import (
    declare_drone_and_namespace_args,
    resolve_drone_and_namespace,
)
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    _, namespace = resolve_drone_and_namespace(context)
    return [
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
        declare_drone_and_namespace_args() + [OpaqueFunction(function=launch_setup)]
    )
