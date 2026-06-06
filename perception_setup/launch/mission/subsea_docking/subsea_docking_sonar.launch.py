"""Subsea docking sonar pipeline.

Wraps the Norbit FLS sonar, line detection RANSAC, and docking position
estimator into a single composable container.

Data sources (via `sim` arg):
  sim:=false  — launches Norbit FLS sonar hardware
  sim:=true   — skips sonar hardware; simulator publishes sonar images
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from auv_setup.launch_arg_common import (
    declare_drone_and_namespace_args,
    resolve_drone_and_namespace,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def _launch_setup(context, *args, **kwargs):
    drone, namespace = resolve_drone_and_namespace(context)

    with open(os.path.join(
        get_package_share_directory('auv_setup'), 'config', 'robots', f'{drone}.yaml',
    )) as f:
        robot_topics = yaml.safe_load(f)['/**']['ros__parameters']['topics']

    sim = LaunchConfiguration('sim').perform(context).lower() == 'true'

    nodes = []

    if sim:
        line_detection_params = {
            'topic.image_sub_topic': 'fls_image/display_mono',
            'topic.line_segments_pub_topic': 'line_detection/line_segments',
            'topic.color_overlay_pub_topic': 'line_detection/visualization',
            'topic.boundary_debug_pub_topic': 'line_detection/boundary_image',
            'topic.boundary_overlay_pub_topic': 'line_detection/boundary_overlay',
            'boundary_detection.threshold': 15,
            'boundary_detection.step': 1.0,
            'boundary_detection.num_rays': 50,
            'boundary_detection.sample_side_length': 5,
            'boundary_detection.angle': 150,
            'boundary_detection.edge_detection': True,
            'boundary_detection.min_dist_from_origin': 60.0,
            'ransac.points_checked': 4,
            'ransac.inlier_threshold': 2.0,
            'ransac.min_remaining_points': 6,
            'ransac.min_inliers': 6,
            'ransac.max_distance': 100.0,
            'mode': 'debug',
            'use_sim_time': False,
        }
    else:
        line_detection_params = {
            'topic.image_sub_topic': '/fls_image/display_mono',
            'topic.line_segments_pub_topic': 'line_detection/line_segments',
            'topic.color_overlay_pub_topic': 'line_detection/visualization',
            'topic.boundary_debug_pub_topic': 'line_detection/boundary_image',
            'topic.boundary_overlay_pub_topic': 'line_detection/boundary_overlay',
            'boundary_detection.threshold': 15,
            'boundary_detection.step': 1.0,
            'boundary_detection.num_rays': 50,
            'boundary_detection.sample_side_length': 5,
            'boundary_detection.angle': 150,
            'boundary_detection.edge_detection': True,
            'boundary_detection.min_dist_from_origin': 30.0,
            'ransac.points_checked': 4,
            'ransac.inlier_threshold': 2.0,
            'ransac.min_remaining_points': 6,
            'ransac.min_inliers': 6,
            'ransac.max_distance': 200.0,
            'mode': 'debug',
            'use_sim_time': False,
        }

    nodes.append(
        ComposableNode(
            package='line_detection_ransac',
            plugin='LineDetectionRansacNode',
            name='line_detection_ransac',
            namespace=namespace,
            parameters=[line_detection_params],
        )
    )

    nodes.append(
        ComposableNode(
            package='docking_position_estimator',
            plugin='vortex::docking_position_estimator::DockingPositionEstimatorNode',
            name='docking_position_estimator',
            namespace=namespace,
            parameters=[{
                'start_mission_service': 'docking_position_estimator/start_mission',
                'send_pose_service': '/docking_position_estimator/docking_pose',
                'line_sub_topic': f'/{namespace}/line_detection/line_segments',
                'pose_sub_topic': f'/{namespace}/{robot_topics["pose"]}',
                'sonar_info_sub_topic': f'/{namespace}/fls/sonar_info',
                'debug_topic': f'/{namespace}/docking_position_debug_viz',
                'odom_frame': f'{namespace}/odom',
                'min_wall_distance_m': 0.5,
                'max_wall_distance_m': 10.0,
                'parallel_heading_angle_threshold_rad': 0.4,
                'perpendicular_heading_angle_threshold_rad': 1.35,
                'min_corner_angle_rad': 1.50,
                'max_corner_angle_rad': 1.70,
                'side_wall_offset_m': 2.5,
                'far_wall_offset_m': 4.0,
                'right_wall_max_y_m': 0.4,
                'far_wall_min_x_m': 0.5,
                'use_left_wall': False,
                'switching_threshold': 0.5,
                'overwrite_prior_waypoints': True,
                'take_priority': True,
                'use_sim_time': False,
            }],
        )
    )

    return [
        ComposableNodeContainer(
            name='sonar_docking_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=nodes,
            output='screen',
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        declare_drone_and_namespace_args()
        + [
            DeclareLaunchArgument(
                'sim',
                default_value='false',
                choices=['true', 'false'],
                description='Run in simulation mode (true) or with real hardware (false)',
            ),
            DeclareLaunchArgument(
                'enable_sonar',
                default_value='true',
                choices=['true', 'false'],
                description='Include the sonar hardware pipeline. Has no effect when sim=true.',
            ),
            DeclareLaunchArgument(
                'vres',
                default_value='512',
                description='Sonar vertical resolution in pixels.',
            ),
            DeclareLaunchArgument(
                'hres',
                default_value='256',
                description='Sonar horizontal resolution in pixels.',
            ),
            DeclareLaunchArgument(
                'horizontal_fov_deg',
                default_value='140.0',
                description='Sonar horizontal field of view in degrees.',
            ),
            DeclareLaunchArgument(
                'range_start',
                default_value='0.0',
                description='Sonar range start in metres.',
            ),
            DeclareLaunchArgument(
                'range_stop',
                default_value='20.0',
                description='Sonar range stop in metres.',
            ),
            DeclareLaunchArgument(
                'datatype_uint8',
                default_value='true',
                description='Use uint8 instead of uint16 for the cartesian sonar image.',
                choices=['true', 'false'],
            ),
            DeclareLaunchArgument(
                'enable_gstreamer',
                default_value='false',
                description='Stream the sonar line detection visualization via GStreamer/RTP.',
            ),
            OpaqueFunction(function=_launch_setup),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('perception_setup'),
                        'launch', 'cameras', 'sonar.launch.py',
                    )
                ),
                launch_arguments={
                    'standalone': 'false',
                    'container_name': 'sonar_docking_container',
                    'enable_gstreamer': LaunchConfiguration('enable_gstreamer'),
                }.items(),
                condition=IfCondition(
                    PythonExpression([
                        '"', LaunchConfiguration('sim'), '" == "false"',
                        ' and ',
                        '"', LaunchConfiguration('enable_sonar'), '" == "true"',
                    ])
                ),
            ),
        ]
    )
