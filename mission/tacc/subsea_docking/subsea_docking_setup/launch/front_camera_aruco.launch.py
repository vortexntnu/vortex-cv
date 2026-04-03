import os

import yaml
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


def load_scenario(scenario, namespace):
    path = os.path.join(
        get_package_share_directory("subsea_docking_setup"),
        "config",
        "scenarios",
        f"{scenario}.yaml",
    )
    with open(path) as f:
        raw = f.read().replace("{ns}", namespace)
    return yaml.safe_load(raw)["scenario"]


def launch_setup(context, *args, **kwargs):
    drone, namespace = resolve_drone_and_namespace(context)
    scenario = context.launch_configurations["scenario"]
    cfg = load_scenario(scenario, namespace)

    perception_share = get_package_share_directory("perception_setup")
    aruco_base_params = os.path.join(
        perception_share, "config", "aruco_detector_params.yaml"
    )

    cam = cfg["front_camera"]
    board = cfg["aruco_board"]

    aruco_node = ComposableNode(
        package="aruco_detector",
        plugin="vortex::aruco_detector::ArucoDetectorNode",
        name="front_aruco_detector",
        namespace=namespace,
        parameters=[
            aruco_base_params,
            {
                "aruco.marker_size": board["marker_size"],
                "board.xDist": board["xDist"],
                "board.yDist": board["yDist"],
                "board.ids": board["ids"],
                "subs.image_topic": cam["image_topic"],
                "subs.camera_info_topic": cam["camera_info_topic"],
                "camera_frame": cam["camera_frame"],
            },
        ],
        extra_arguments=[{"use_intra_process_comms": cfg["use_camera_driver"]}],
    )

    nodes = [aruco_node]

    if cfg["use_camera_driver"]:
        camera_params = PathJoinSubstitution(
            [FindPackageShare("perception_setup"), "config", "front_camera_params.yaml"]
        )
        blackfly_config = PathJoinSubstitution(
            [FindPackageShare("perception_setup"), "config", "blackfly_s_params.yaml"]
        )
        calib_url = f"file://{os.path.join(perception_share, 'config', 'front_camera_calib_downscale.yaml')}"

        driver_node = ComposableNode(
            package="spinnaker_camera_driver",
            plugin="spinnaker_camera_driver::CameraDriver",
            name="front_camera",
            namespace=namespace,
            parameters=[
                camera_params,
                {
                    "parameter_file": blackfly_config,
                    "serial_number": cam["serial_number"],
                    "camerainfo_url": calib_url,
                },
            ],
            remappings=[
                ("~/control", "/exposure_control/control"),
                ("/flir_camera/image_raw", cam["image_topic"]),
                ("/flir_camera/camera_info", cam["camera_info_topic"]),
            ],
            extra_arguments=[{"use_intra_process_comms": False}],
        )
        nodes.insert(0, driver_node)

    container = ComposableNodeContainer(
        name="front_camera_aruco_container",
        namespace=namespace,
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=nodes,
        output="screen",
        arguments=["--ros-args", "--log-level", "error"],
    )

    return [container]


def generate_launch_description():
    return LaunchDescription(
        declare_drone_and_namespace_args()
        + [
            DeclareLaunchArgument(
                "scenario",
                default_value="sim",
                description="Scenario to load: sim, real_world",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
