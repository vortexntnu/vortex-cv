import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    cfg = os.path.join(
        get_package_share_directory('valve_detection'),
        'config',
        'valve_detection_params.yaml',
    )

    return LaunchDescription(
        [
            # Input topics
            DeclareLaunchArgument(
                'depth_image_sub_topic',
                default_value='/camera/camera/depth/image_rect_raw',
                description='Depth image topic',
            ),
            DeclareLaunchArgument(
                'detections_sub_topic',
                default_value='/obb_detections_output',
                description='YOLO detections topic',
            ),
            DeclareLaunchArgument(
                'depth_image_info_topic',
                default_value='/camera/camera/depth/camera_info',
                description='Depth camera info topic',
            ),
            DeclareLaunchArgument(
                'color_image_info_topic',
                default_value='/yolo_obb_encoder/internal/resize/camera_info',
                description='Color camera info topic (from DNN encoder)',
            ),
            # Frame IDs
            DeclareLaunchArgument(
                'depth_frame_id',
                default_value='front_camera_depth_optical',
                description='Depth camera optical frame ID (without drone prefix)',
            ),
            DeclareLaunchArgument(
                'color_frame_id',
                default_value='front_camera_color_optical',
                description='Color camera optical frame ID (without drone prefix)',
            ),
            # Output topic
            DeclareLaunchArgument(
                'landmarks_pub_topic',
                default_value='/valve_landmarks',
                description='Output valve landmarks topic',
            ),
            DeclareLaunchArgument(
                'output_frame_id',
                default_value='front_camera_depth_optical',
                description='Output frame ID for published poses (without drone prefix)',
            ),
            # Drone identifier
            DeclareLaunchArgument(
                'drone',
                default_value='nautilus',
                description='Robot name, prepended to TF frame IDs (e.g. moby, orca)',
            ),
            # Detection processing
            DeclareLaunchArgument(
                'undistort_detections',
                default_value='false',
                description='Undistort bounding-box detections using color camera distortion',
            ),
            # Debug visualization
            DeclareLaunchArgument(
                'debug_visualize',
                default_value='true',
                description='Enable all debug visualization topics',
            ),
            # Node container with parameters from launch arguments
            ComposableNodeContainer(
                name='valve_detection_container',
                namespace='',
                package='rclcpp_components',
                executable='component_container_mt',
                composable_node_descriptions=[
                    ComposableNode(
                        package='valve_detection',
                        plugin='valve_detection::ValvePoseNode',
                        name='valve_pose_node',
                        parameters=[
                            cfg,
                            {
                                'depth_image_sub_topic': LaunchConfiguration('depth_image_sub_topic'),
                                'detections_sub_topic': LaunchConfiguration('detections_sub_topic'),
                                'depth_image_info_topic': LaunchConfiguration('depth_image_info_topic'),
                                'color_image_info_topic': LaunchConfiguration('color_image_info_topic'),
                                'depth_frame_id': LaunchConfiguration('depth_frame_id'),
                                'color_frame_id': LaunchConfiguration('color_frame_id'),
                                'landmarks_pub_topic': LaunchConfiguration('landmarks_pub_topic'),
                                'output_frame_id': LaunchConfiguration('output_frame_id'),
                                'drone': LaunchConfiguration('drone'),
                                'undistort_detections': LaunchConfiguration('undistort_detections'),
                                'debug_visualize': LaunchConfiguration('debug_visualize'),
                            },
                        ],
                    )
                ],
                output='screen',
            ),
        ]
    )
