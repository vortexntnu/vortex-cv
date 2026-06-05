"""
Acoustics bearing test — run this alongside the simulation and FSM.

Starts:
  1. acoustics_bearing_server   → exposes /nautilus/acoustics_bearing_direction
  2. test_acoustics_direction_node → publishes noisy bearings toward the pipeline start

Then launch the FSM separately:
  ros2 launch perception_setup pipeline_inspection_fsm.launch.py start_above_pipe:=true

And trigger the mission:
  ros2 service call /nautilus/pipeline_inspection_fsm/start_mission std_srvs/srv/Trigger

Or test the action directly:
  ros2 action send_goal /nautilus/acoustics_bearing_direction \
    vortex_msgs/action/CollectBearingDirection "{timeout_sec: 10.0, distance: 5.0}"
"""

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
    _, namespace = resolve_drone_and_namespace(context)

    config = os.path.join(
        get_package_share_directory("bearing_direction_server"),
        "config", "bearing_direction_server.yaml",
    )

    acoustics_server = Node(
        package="bearing_direction_server",
        executable="acoustics_bearing_server",
        name="acoustics_bearing_server",
        namespace=namespace,
        output="screen",
        parameters=[
            config,
            {
                "action_name": "acoustics_bearing_direction",
                "topics.bearing_measurements": "acoustics/bearing_measurements",
            },
        ],
    )

    test_publisher = Node(
        package="bearing_direction_server",
        executable="test_acoustics_direction_node",
        name="test_acoustics_direction_node",
        namespace=namespace,
        output="screen",
        parameters=[{
            "target.x": 1.777425535555405,
            "target.y": -0.053997583427242916,
            "target.z": 5.301166999942316,
            "noise_std_deg": 3.0,
            "topics.odom": "/nautilus/odom",
            "topics.bearing_measurements": "acoustics/bearing_measurements",
            "publish_rate_hz": 10.0,
        }],
    )

    return [acoustics_server, test_publisher]


def generate_launch_description():
    return LaunchDescription(
        declare_drone_and_namespace_args()
        + [OpaqueFunction(function=launch_setup)]
    )
