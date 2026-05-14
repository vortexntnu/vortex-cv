import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context):
    drone = LaunchConfiguration('drone').perform(context)

    config = os.path.join(
        get_package_share_directory("docking_camera_yolo_direction_waypoint"),
        "config",
        "docking_camera_yolo_direction_waypoint.yaml",
    )

    return [
        Node(
            package="docking_camera_yolo_direction_waypoint",
            executable="docking_camera_yolo_direction_waypoint_node",
            name="docking_camera_yolo_direction_waypoint",
            output="screen",
            parameters=[
                config,
                {
                    "use_sim_time": True,
                    "waypoint_distance": LaunchConfiguration("waypoint_distance"),
                    "camera_info_sub_topic": f"/{drone}/front_camera/camera_info",
                    "landmarks_pub_topic": f"/{drone}/landmarks",
                    "odom_frame": f"{drone}/odom",
                },
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "drone",
            default_value="nautilus",
            description="Robot name, used as topic namespace prefix",
        ),
        DeclareLaunchArgument(
            "waypoint_distance",
            default_value="2.0",
            description="Distance [m] ahead of the camera to place the waypoint along the target yaw.",
        ),
        OpaqueFunction(function=_launch_setup),
    ])
