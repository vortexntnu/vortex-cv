from auv_setup.launch_arg_common import (
    declare_drone_and_namespace_args,
    resolve_drone_and_namespace,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):
    drone, namespace = resolve_drone_and_namespace(context)
    use_sim = LaunchConfiguration("sim").perform(context).lower() == "true"

    nodes = []

    # Norbit FLS sonar interface — real hardware only
    if not use_sim:
        container = ComposableNodeContainer(
            name="sonar_container",
            namespace=namespace,
            package="rclcpp_components",
            executable="component_container_mt",
            composable_node_descriptions=[
                ComposableNode(
                    package="norbit_fls_ros_interface",
                    plugin="NorbitFLSRosInterface",
                    name="norbit_fls_ros_interface_node",
                    namespace=namespace,
                    parameters=[
                        {
                            "raw_sonar_image_pub_topic": "fls_image/image",
                            "cartesian_sonar_image_pub_topic": "fls_image/display_mono",
                            "norbit_sonar_info_pub_topic": "fls/norbit_sonar_info",
                            "sonar_info_pub_topic": "fls/sonar_info",
                            "frame_id": "fls_frame",
                            "connection_params.ip": "10.0.0.7",
                            "connection_params.bathy_port": 2210,
                            "connection_params.water_column_port": 2211,
                            "connection_params.snippet_sidescan_port": 2212,
                            "connection_params.cmd_port": 2209,
                            "connection_params.cmd_timeout_sec": 2.0,
                            "sonar_settings.mode": "Bathy",
                            "sonar_settings.vres": 512,
                            "sonar_settings.hres": 256,
                            "sonar_settings.range_start": 0.0,
                            "sonar_settings.range_stop": 2.0,
                        }
                    ],
                    extra_arguments=[{"use_intra_process_comms": True}],
                ),
            ],
            output="screen",
            arguments=["--ros-args", "--log-level", "error"],
        )
        nodes.append(container)

    # line_detection_ransac
    if use_sim:
        line_detection_params = {
            "topic.image_sub_topic": "/nautilus/fls_image/display_mono",
            "topic.line_segments_pub_topic": "line_detection/line_segments",
            "topic.color_overlay_pub_topic": "line_detection/visualization",
            "topic.boundary_debug_pub_topic": "line_detection/boundary_image",
            "topic.boundary_overlay_pub_topic": "line_detection/boundary_overlay",
            "boundary_detection.threshold": 15,
            "boundary_detection.step": 1.0,
            "boundary_detection.num_rays": 50,
            "boundary_detection.sample_side_length": 5,
            "boundary_detection.angle": 150,
            "boundary_detection.edge_detection": True,
            "boundary_detection.min_dist_from_origin": 60.0,
            "ransac.points_checked": 4,
            "ransac.inlier_threshold": 2.0,
            "ransac.min_remaining_points": 6,
            "ransac.min_inliers": 6,
            "ransac.max_distance": 100.0,
            "mode": "debug",
            "use_sim_time": False,
        }
    else:
        line_detection_params = {
            "topic.image_sub_topic": "fls_image/display_mono",
            "topic.line_segments_pub_topic": "line_detection/line_segments",
            "topic.color_overlay_pub_topic": "line_detection/visualization",
            "topic.boundary_debug_pub_topic": "line_detection/boundary_image",
            "topic.boundary_overlay_pub_topic": "line_detection/boundary_overlay",
            "boundary_detection.threshold": 15,
            "boundary_detection.step": 1.0,
            "boundary_detection.num_rays": 50,
            "boundary_detection.sample_side_length": 5,
            "boundary_detection.angle": 150,
            "boundary_detection.edge_detection": True,
            "boundary_detection.min_dist_from_origin": 30.0,
            "ransac.points_checked": 4,
            "ransac.inlier_threshold": 2.0,
            "ransac.min_remaining_points": 6,
            "ransac.min_inliers": 6,
            "ransac.max_distance": 200.0,
            "mode": "debug",
            "use_sim_time": False,
        }

    line_detection_node = Node(
        package="line_detection_ransac",
        executable="line_detection_ransac_node",
        name="line_detection_ransac",
        namespace=namespace,
        output="screen",
        parameters=[line_detection_params],
    )
    nodes.append(line_detection_node)

    # docking_position_estimator
    docking_estimator_node = Node(
        package="docking_position_estimator",
        executable="docking_position_estimator_node",
        name="docking_position_estimator",
        namespace=namespace,
        output="screen",
        parameters=[
            {
                "start_mission_service": "docking_position_estimator/start_mission",
                "send_pose_service": "/docking_position_estimator/docking_pose",
                "line_sub_topic": f"/{namespace}/line_detection/line_segments",
                "pose_sub_topic": f"/{namespace}/pose",
                "sonar_info_sub_topic": f"/{namespace}/fls/sonar_info",
                "debug_topic": f"/{namespace}/docking_position_debug_viz",
                "odom_frame": f"{namespace}/odom",
                "min_wall_distance_m": 0.5,
                "max_wall_distance_m": 10.0,
                "parallel_heading_angle_threshold_rad": 0.4,
                "perpendicular_heading_angle_threshold_rad": 1.35,
                "min_corner_angle_rad": 1.50,
                "max_corner_angle_rad": 1.70,
                "side_wall_offset_m": 2.5,
                "far_wall_offset_m": 4.0,
                "right_wall_max_y_m": 0.4,
                "far_wall_min_x_m": 0.5,
                "use_left_wall": False,
                "switching_threshold": 0.5,
                "overwrite_prior_waypoints": True,
                "take_priority": True,
                "use_sim_time": False,
            }
        ],
    )
    nodes.append(docking_estimator_node)

    return nodes


def generate_launch_description():
    return LaunchDescription(
        declare_drone_and_namespace_args()
        + [
            DeclareLaunchArgument(
                "sim",
                default_value="false",
                choices=["true", "false"],
                description="Run in simulation mode (true) or with real hardware (false)",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
