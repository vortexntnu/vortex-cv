"""Ultralytics YOLO classification inference pipeline.

Args (shared interface style):
  model_input_image_topic  — input image topic
  model_file_path          — path to .pt model file
  device                   — inference device

Classify-specific:
  output_class_topic, imgsz, verbose
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

    image_topic = LaunchConfiguration('model_input_image_topic').perform(context)
    model_file_path = LaunchConfiguration('model_file_path').perform(context)
    output_class_topic = LaunchConfiguration('output_class_topic').perform(context)
    imgsz = int(LaunchConfiguration('imgsz').perform(context))
    verbose = LaunchConfiguration('verbose').perform(context).lower() == 'true'

    node_name = LaunchConfiguration('node_name').perform(context)

    node = Node(
        package='yolo_classify',
        executable='classifier_node',
        name=node_name,
        namespace='yolo',
        output='screen',
        parameters=[
            {
                'model_path': model_file_path,
                'device': device,
                'input_topic': image_topic,
                'output_class_topic': output_class_topic,
                'imgsz': imgsz,
                'verbose': verbose,
            }
        ],
    )

    return [node]


def generate_launch_description():
    pkg_dir = get_package_share_directory('perception_setup')

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'node_name',
                default_value='classifier_node',
                description='ROS node name for the classification node.',
            ),
            DeclareLaunchArgument(
                'model_input_image_topic',
                default_value='/pipeline/camera/segmentation_mask',
            ),
            DeclareLaunchArgument(
                'model_file_path',
                default_value=os.path.join(pkg_dir, 'models', 'best.pt'),
            ),
            DeclareLaunchArgument(
                'output_class_topic',
                default_value='/classification_result',
            ),
            DeclareLaunchArgument(
                'device',
                default_value='cpu',
                description="Inference device: 'cpu', GPU index, 'cuda', 'cuda:N', or 'mps'",
            ),
            DeclareLaunchArgument(
                'imgsz',
                default_value='640',
                description='Input image size passed to YOLO inference',
            ),
            DeclareLaunchArgument(
                'verbose',
                default_value='false',
                description='Enable YOLO per-inference console output',
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
