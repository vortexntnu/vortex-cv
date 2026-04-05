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

    aruco_base_params = os.path.join(
        get_package_share_directory("aruco_detector"),
        "config",
        "aruco_detector_params.yaml",
    )

    node = Node(
        package="aruco_detector",
        executable="aruco_detector_node",
        name="down_aruco_detector",
        namespace=namespace,
        parameters=[
            aruco_base_params,
            {
                "aruco.marker_size": 0.150,
                "board.xDist": 0.430,
                "board.yDist": 0.830,
                "board.ids": [28, 7, 96, 19],
                "subs.image_topic": f"/{namespace}/down_camera/image_color",
                "subs.camera_info_topic": f"/{namespace}/down_camera/camera_info",
                "out_tf_frame": f"{namespace}/downwards_camera_optical",
            },
        ],
        output="screen",
    )

    return [node]


def generate_launch_description():
    return LaunchDescription(
        declare_drone_and_namespace_args() + [OpaqueFunction(function=launch_setup)]
    )
