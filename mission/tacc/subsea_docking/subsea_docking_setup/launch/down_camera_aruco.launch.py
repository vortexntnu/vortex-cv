import os

from ament_index_python.packages import get_package_share_directory
from auv_setup.launch_arg_common import (
    declare_drone_and_namespace_args,
    resolve_drone_and_namespace,
)
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    drone, namespace = resolve_drone_and_namespace(context)

    perception_setup_share = get_package_share_directory("perception_setup")

    down_camera_params_file = PathJoinSubstitution(
        [FindPackageShare("perception_setup"), "config", "downwards_cam_params.yaml"]
    )

    blackfly_s_config_file = PathJoinSubstitution(
        [FindPackageShare("perception_setup"), "config", "blackfly_s_params.yaml"]
    )

    down_camera_calib_url = f"file://{os.path.join(perception_setup_share, 'config', 'downwards_cam_calib.yaml')}"

    aruco_params_file = os.path.join(
        perception_setup_share,
        "config",
        "aruco_detector_params.yaml",
    )

    container = ComposableNodeContainer(
        name="down_camera_aruco_container",
        namespace=namespace,
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="spinnaker_camera_driver",
                plugin="spinnaker_camera_driver::CameraDriver",
                name="down_camera",
                namespace=namespace,
                parameters=[
                    down_camera_params_file,
                    {
                        "parameter_file": blackfly_s_config_file,
                        "serial_number": "23494258",
                        "camerainfo_url": down_camera_calib_url,
                    },
                ],
                remappings=[
                    ("~/control", "/exposure_control/control"),
                    ("/flir_camera/image_raw", f"/{namespace}/down_camera/image_raw"),
                    (
                        "/flir_camera/camera_info",
                        f"/{namespace}/down_camera/camera_info",
                    ),
                ],
                extra_arguments=[{"use_intra_process_comms": False}],
            ),
            ComposableNode(
                package="aruco_detector",
                plugin="vortex::aruco_detector::ArucoDetectorNode",
                name="aruco_detector",
                namespace=namespace,
                parameters=[
                    aruco_params_file,
                    {
                        "subs.image_topic": f"/{namespace}/down_camera/image_raw",
                        "subs.camera_info_topic": f"/{namespace}/down_camera/camera_info",
                        "camera_frame": f"{namespace}/down_camera_link",
                    },
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
        output="screen",
        arguments=["--ros-args", "--log-level", "error"],
    )

    return [container]


def generate_launch_description():
    return LaunchDescription(
        declare_drone_and_namespace_args()
        + [
            OpaqueFunction(function=launch_setup),
        ]
    )
