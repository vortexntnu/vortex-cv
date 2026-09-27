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
                description='Absolute path to calibration YAML',
            ),
            ComposableNodeContainer(
                name='camera_info_sync_container',
                namespace='',
                package='rclcpp_components',
                executable='component_container_mt',
                composable_node_descriptions=[
                    ComposableNode(
                        package='vortex_cv_util_nodes',
                        plugin='vortex_cv_util_nodes::CameraInfoSync',
                        name='camera_info_sync',
                        parameters=[
                            {
                                'camera_info_file': LaunchConfiguration(
                                    'camera_info_file'
                                ),
                                'camera_info_topic': '/camera/camera/color/camera_info',
                                'image_topic': '/camera/camera/color/image_raw',
                            }
                        ],
                    ),
                ],
                output='screen',
            ),
        ]
    )
