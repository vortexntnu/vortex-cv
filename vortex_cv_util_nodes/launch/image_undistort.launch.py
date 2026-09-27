from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'camera_info_file',
                default_value='',
                description='Absolute path to calibration YAML; leave empty to use camera_info_topic instead',
            ),
            DeclareLaunchArgument(
                'enable_undistort',
                default_value='true',
                description='true = undistort, false = passthrough',
            ),
            ComposableNodeContainer(
                name='image_undistort_container',
                namespace='',
                package='rclcpp_components',
                executable='component_container_mt',
                composable_node_descriptions=[
                    ComposableNode(
                        package='vortex_cv_util_nodes',
                        plugin='vortex_cv_util_nodes::ImageUndistort',
                        name='image_undistort',
                        parameters=[
                            {
                                'image_topic': '/camera/camera/color/image_raw',
                                'camera_info_file': LaunchConfiguration(
                                    'camera_info_file'
                                ),
                                'camera_info_topic': '',
                                'raw_camera_info_topic': '/camera/camera/color/camera_info',
                                'output_image_topic': '/realsense_d555/color/image_rect',
                                'output_camera_info_topic': '/realsense_d555/color/camera_info',
                                'enable_undistort': LaunchConfiguration(
                                    'enable_undistort'
                                ),
                                'image_qos': 'sensor_data',
                            }
                        ],
                    ),
                ],
                output='screen',
            ),
        ]
    )
