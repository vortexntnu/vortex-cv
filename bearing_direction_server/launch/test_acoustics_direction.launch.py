from auv_setup.launch_arg_common import (
    declare_drone_and_namespace_args,
    resolve_drone_and_namespace,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    _, namespace = resolve_drone_and_namespace(context)

    return [
        Node(
            package="bearing_direction_server",
            executable="test_acoustics_direction_node",
            name="test_acoustics_direction_node",
            namespace=namespace,
            output="screen",
            parameters=[
                {
                    # Pipeline start position (world/odom frame)
                    "target.x": float(LaunchConfiguration('target_x').perform(context)),
                    "target.y": float(LaunchConfiguration('target_y').perform(context)),
                    "target.z": float(LaunchConfiguration('target_z').perform(context)),
                    # Angular noise std — tune to match real acoustic sensor
                    "noise_std_deg": 3.0,
                    "topics.odom": f"/{namespace}/odom",
                    "topics.bearing_measurements": "acoustics/bearing_measurements",
                    "publish_rate_hz": 10.0,
                }
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        declare_drone_and_namespace_args()
        + [
            DeclareLaunchArgument(
                'target_x',
                default_value='1.777425535555405',
                description='Pipeline target X in odom frame',
            ),
            DeclareLaunchArgument(
                'target_y',
                default_value='-0.053997583427242916',
                description='Pipeline target Y in odom frame',
            ),
            DeclareLaunchArgument(
                'target_z',
                default_value='5.301166999942316',
                description='Pipeline target Z in odom frame',
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
