"""Ultralytics YOLO-OBB inference pipeline.

Shared launch-arg interface with isaac_ros/isaac_ros_yolo_obb.launch.py:
  image_topic, camera_info_topic (declared but unused), image_width/height (unused),
  model_file_path, detections_topic, annotated_image_topic, confidence_threshold

Ultralytics-specific:
  device
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _validate_device(device: str) -> None:
    if device in ('cpu', 'cuda', 'mps'):
        return
    if device.isdigit():
        return
    if device.startswith('cuda:') and device.split(':')[1].isdigit():
        return
    raise RuntimeError(
        f"Invalid device '{device}'. Use 'cpu', GPU index (0,1,...), 'cuda', 'cuda:N', or 'mps'."
    )


def _launch_setup(context, *args, **kwargs):
    device = LaunchConfiguration('device').perform(context)
    _validate_device(device)

    image_topic = LaunchConfiguration('image_topic').perform(context)
    model_file_path = LaunchConfiguration('model_file_path').perform(context)
    detections_topic = LaunchConfiguration('detections_topic').perform(context)
    annotated_image_topic = LaunchConfiguration('annotated_image_topic').perform(context)
    confidence_threshold = float(LaunchConfiguration('confidence_threshold').perform(context))
    visualize = LaunchConfiguration('visualize').perform(context).lower() == 'true'

    yolo_node = Node(
        package='yolo_obb_object_detection',
        executable='yolo_obb_object_detection_node',
        name='yolo_obb_object_detection',
        output='screen',
        parameters=[{
            'device': device,
            'yolo_model': model_file_path,
            'model_conf': confidence_threshold,
            'color_image_sub_topic': image_topic,
            'yolo_detections_pub_topic': detections_topic,
            'yolo_annotated_pub_topic': annotated_image_topic if visualize else '',
        }],
    )

    return [yolo_node]


def generate_launch_description():
    pkg_dir = get_package_share_directory('perception_setup')

    return LaunchDescription([
        DeclareLaunchArgument(
            'image_topic',
            default_value='/camera/camera/color/image_raw',
        ),
        # Declared for interface parity with isaac_ros/isaac_ros_yolo_obb.launch.py; not used.
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='/camera/camera/color/camera_info',
        ),
        DeclareLaunchArgument('image_width', default_value='896'),
        DeclareLaunchArgument('image_height', default_value='504'),
        DeclareLaunchArgument(
            'model_file_path',
            default_value=os.path.join(
                pkg_dir, 'models', 'best_valve_handle_real_and_sim.pt'
            ),
        ),
        DeclareLaunchArgument(
            'detections_topic',
            default_value='/yolo_obb/detections',
        ),
        DeclareLaunchArgument(
            'annotated_image_topic',
            default_value='/yolo_obb/annotated_image',
        ),
        DeclareLaunchArgument('confidence_threshold', default_value='0.25'),
        DeclareLaunchArgument(
            'device',
            default_value='0',
            description="Inference device: 'cpu', GPU index, 'cuda', 'cuda:N', or 'mps'",
        ),
        DeclareLaunchArgument(
            'visualize',
            default_value='true',
            description='Publish annotated images on annotated_image_topic',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
