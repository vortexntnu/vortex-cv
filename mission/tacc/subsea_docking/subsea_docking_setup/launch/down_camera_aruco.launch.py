from auv_setup.launch_arg_common import (
    declare_drone_and_namespace_args,
    resolve_drone_and_namespace,
)
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    drone, namespace = resolve_drone_and_namespace(context)

    node = Node(
        package="aruco_detector",
        executable="aruco_detector_node",
        name="down_aruco_detector",
        namespace=namespace,
        output="screen",
        parameters=[
            {
                "subs.image_topic": f"/{namespace}/down_camera/image_color",
                "subs.camera_info_topic": f"/{namespace}/down_camera/camera_info",
                "pubs.aruco_image": "/aruco_detector/image_down",
                "pubs.aruco_poses": "/aruco_detector/markers_down",
                "pubs.board_pose": "/aruco_detector/board_down",
                "pubs.landmarks": f"/{namespace}/landmarks",
                "logger_service_name": "/toggle_marker_logger",
                "detect_board": True,
                "visualize": True,
                "log_markers": False,
                "publish_detections": True,
                "publish_landmarks": True,
                "aruco.marker_size": 0.150,
                "aruco.dictionary": "DICT_ARUCO_ORIGINAL",
                "board.xDist": 0.430,
                "board.yDist": 0.830,
                "board.ids": [28, 7, 96, 19],
                "enu_ned_rotation": True,
                "out_tf_frame": f"{namespace}/downwards_camera_optical",
            }
        ],
    )

    return [node]


def generate_launch_description():
    return LaunchDescription(
        declare_drone_and_namespace_args() + [OpaqueFunction(function=launch_setup)]
    )
