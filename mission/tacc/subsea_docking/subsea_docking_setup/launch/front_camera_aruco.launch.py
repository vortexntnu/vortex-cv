import os

from ament_index_python.packages import get_package_share_directory
from auv_setup.launch_arg_common import (
    declare_drone_and_namespace_args,
    resolve_drone_and_namespace,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    drone, namespace = resolve_drone_and_namespace(context)
    use_sim = context.launch_configurations["use_sim"].lower() == "true"
    board = context.launch_configurations["board"]

    perception_setup_share = get_package_share_directory("perception_setup")

    aruco_base_params = os.path.join(
        perception_setup_share, "config", "aruco_detector_params.yaml"
    )

    aruco_board_params = os.path.join(
        get_package_share_directory("subsea_docking_setup"),
        "config",
        f"aruco_{board}.yaml",
    )

    if use_sim:
        image_topic = f"/{namespace}/front_camera"
        camera_info_topic = f"/{namespace}/front_camera/camera_info"

        container = ComposableNodeContainer(
            name="front_camera_aruco_container",
            namespace=namespace,
            package="rclcpp_components",
            executable="component_container_mt",
            composable_node_descriptions=[
                ComposableNode(
                    package="aruco_detector",
                    plugin="vortex::aruco_detector::ArucoDetectorNode",
                    name="aruco_detector",
                    namespace=namespace,
                    parameters=[
                        aruco_base_params,
                        aruco_board_params,
                        {
                            "subs.image_topic": image_topic,
                            "subs.camera_info_topic": camera_info_topic,
                            "camera_frame": f"{namespace}/front_camera_link",
                        },
                    ],
                    extra_arguments=[{"use_intra_process_comms": False}],
                ),
            ],
            output="screen",
            arguments=["--ros-args", "--log-level", "error"],
        )
    else:
        front_camera_params_file = PathJoinSubstitution(
            [FindPackageShare("perception_setup"), "config", "front_camera_params.yaml"]
        )

        blackfly_s_config_file = PathJoinSubstitution(
            [FindPackageShare("perception_setup"), "config", "blackfly_s_params.yaml"]
        )

        front_camera_calib_url = f"file://{os.path.join(perception_setup_share, 'config', 'front_camera_calib_downscale.yaml')}"

        image_topic = f"/{namespace}/front_camera/image_raw"
        camera_info_topic = f"/{namespace}/front_camera/camera_info"

        container = ComposableNodeContainer(
            name="front_camera_aruco_container",
            namespace=namespace,
            package="rclcpp_components",
            executable="component_container_mt",
            composable_node_descriptions=[
                ComposableNode(
                    package="spinnaker_camera_driver",
                    plugin="spinnaker_camera_driver::CameraDriver",
                    name="front_camera",
                    namespace=namespace,
                    parameters=[
                        front_camera_params_file,
                        {
                            "parameter_file": blackfly_s_config_file,
                            "serial_number": "23494259",
                            "camerainfo_url": front_camera_calib_url,
                        },
                    ],
                    remappings=[
                        ("~/control", "/exposure_control/control"),
                        ("/flir_camera/image_raw", image_topic),
                        ("/flir_camera/camera_info", camera_info_topic),
                    ],
                    extra_arguments=[{"use_intra_process_comms": False}],
                ),
                ComposableNode(
                    package="aruco_detector",
                    plugin="vortex::aruco_detector::ArucoDetectorNode",
                    name="aruco_detector",
                    namespace=namespace,
                    parameters=[
                        aruco_base_params,
                        aruco_board_params,
                        {
                            "subs.image_topic": image_topic,
                            "subs.camera_info_topic": camera_info_topic,
                            "camera_frame": f"{namespace}/front_camera_link",
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
            DeclareLaunchArgument(
                "use_sim",
                default_value="false",
                description="Skip camera driver and subscribe to sim topics",
            ),
            DeclareLaunchArgument(
                "board",
                default_value="tacc",
                description="ArUco board config: tacc or vortex_plate",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
