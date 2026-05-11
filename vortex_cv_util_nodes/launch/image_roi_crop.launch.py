from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'enable_crop',
                default_value='true',
                description='true = crop to ROI and rescale camera info, false = passthrough',
            ),
            ComposableNodeContainer(
                name='image_roi_crop_container',
                namespace='',
                package='rclcpp_components',
                executable='component_container_mt',
                composable_node_descriptions=[
                    ComposableNode(
                        package='vortex_cv_util_nodes',
                        plugin='vortex_cv_util_nodes::ImageRoiCrop',
                        name='image_roi_crop',
                        parameters=[
                            {
                                'image_topic': '/camera/camera/depth/image_rect_raw',
                                'camera_info_topic': '/camera/camera/depth/camera_info',
                                'output_image_topic': '/camera/camera/depth/image_cropped',
                                'output_camera_info_topic': '/camera/camera/depth/camera_info_cropped',
                                'enable_crop': LaunchConfiguration('enable_crop'),
                                'crop.x_offset': 260,
                                'crop.y_offset': 190,
                                'crop.width': 485,
                                'crop.height': 245,
                            }
                        ],
                    ),
                ],
                output='screen',
            ),
        ]
    )
