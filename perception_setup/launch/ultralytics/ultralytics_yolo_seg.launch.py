"""Ultralytics YOLO segmentation inference pipeline.

Args (shared interface style):
  model_input_image_topic  — input image topic
  camera_info_topic        — input camera info topic
  model_file_path          — path to .pt model file
  device                   — inference device

Segmentation-specific:
  output_bbox_topic, output_mask_topic, output_debug_topic,
  output_camera_info_topic, pub_bbox, pub_mask, pub_debug,
  imgsz, confidence_threshold, max_detections, compile, verbose
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
    camera_info_topic = LaunchConfiguration('camera_info_topic').perform(context)
    model_file_path = LaunchConfiguration('model_file_path').perform(context)
    output_bbox_topic = LaunchConfiguration('output_bbox_topic').perform(context)
    output_mask_topic = LaunchConfiguration('output_mask_topic').perform(context)
    output_debug_topic = LaunchConfiguration('output_debug_topic').perform(context)
    output_camera_info_topic = LaunchConfiguration('output_camera_info_topic').perform(context)
    pub_bbox = LaunchConfiguration('pub_bbox').perform(context).lower() == 'true'
    pub_mask = LaunchConfiguration('pub_mask').perform(context).lower() == 'true'
    pub_debug = LaunchConfiguration('pub_debug').perform(context).lower() == 'true'
    pub_mask_overlay = LaunchConfiguration('pub_mask_overlay').perform(context).lower() == 'true'
    output_mask_overlay_topic = LaunchConfiguration('output_mask_overlay_topic').perform(context)
    imgsz = int(LaunchConfiguration('imgsz').perform(context))
    confidence_threshold = float(LaunchConfiguration('confidence_threshold').perform(context))
    max_detections = int(LaunchConfiguration('max_detections').perform(context))
    compile_model = LaunchConfiguration('compile').perform(context).lower() == 'true'
    verbose = LaunchConfiguration('verbose').perform(context).lower() == 'true'

    node_name = LaunchConfiguration('node_name').perform(context)

    node = Node(
        package='yolo_segmentation',
        executable='yolo_seg_node',
        name=node_name,
        output='screen',
        parameters=[{
            'input_topic': image_topic,
            'input_camera_info_topic': camera_info_topic,
            'output_bbox_topic': output_bbox_topic,
            'output_mask_topic': output_mask_topic,
            'output_debug_topic': output_debug_topic,
            'output_camera_info_topic': output_camera_info_topic,
            'pub_bbox': pub_bbox,
            'pub_mask': pub_mask,
            'pub_debug': pub_debug,
            'pub_mask_overlay': pub_mask_overlay,
            'output_mask_overlay_topic': output_mask_overlay_topic,
            'model_path': model_file_path,
            'device': device,
            'imgsz': imgsz,
            'confidence_threshold': confidence_threshold,
            'max_detections': max_detections,
            'compile': compile_model,
            'verbose': verbose,
        }],
    )

    return [node]


def generate_launch_description():
    pkg_dir = get_package_share_directory('perception_setup')

    return LaunchDescription([
        DeclareLaunchArgument(
            'node_name',
            default_value='yolo_segmentation_node',
            description='ROS node name for the segmentation node.',
        ),
        DeclareLaunchArgument(
            'model_input_image_topic',
            default_value='/camera/camera/color/image_raw',
        ),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='/camera/camera/color/camera_info',
        ),
        DeclareLaunchArgument(
            'model_file_path',
            default_value=os.path.join(pkg_dir, 'models', 'best.pt'),
        ),
        DeclareLaunchArgument(
            'output_bbox_topic',
            default_value='/pipeline/camera/bboxes',
        ),
        DeclareLaunchArgument(
            'output_mask_topic',
            default_value='/pipeline/camera/segmentation_mask',
        ),
        DeclareLaunchArgument(
            'output_debug_topic',
            default_value='/pipeline/camera/segmentation_debug',
        ),
        DeclareLaunchArgument(
            'output_camera_info_topic',
            default_value='/pipeline/camera/camera_info',
        ),
        DeclareLaunchArgument(
            'pub_bbox',
            default_value='true',
            description='Publish bounding boxes',
        ),
        DeclareLaunchArgument(
            'pub_mask',
            default_value='true',
            description='Publish segmentation mask',
        ),
        DeclareLaunchArgument(
            'pub_debug',
            default_value='true',
            description='Publish debug annotated image',
        ),
        DeclareLaunchArgument(
            'pub_mask_overlay',
            default_value='false',
            description='Publish alpha-blended red overlay of mask on original image.',
        ),
        DeclareLaunchArgument(
            'output_mask_overlay_topic',
            default_value='/pipeline/camera/segmentation_overlay',
        ),
        DeclareLaunchArgument(
            'device',
            default_value='cuda',
            description="Inference device: 'cpu', GPU index, 'cuda', 'cuda:N', or 'mps'",
        ),
        DeclareLaunchArgument(
            'imgsz',
            default_value='640',
            description='Longest side input size; shorter side scaled proportionally and padded to stride multiple',
        ),
        DeclareLaunchArgument(
            'confidence_threshold',
            default_value='0.3',
        ),
        DeclareLaunchArgument(
            'max_detections',
            default_value='1',
        ),
        DeclareLaunchArgument(
            'compile',
            default_value='false',
            description='Compile model with torch.compile for faster inference',
        ),
        DeclareLaunchArgument(
            'verbose',
            default_value='false',
            description='Print Ultralytics per-frame output (speed, detections, image size)',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
