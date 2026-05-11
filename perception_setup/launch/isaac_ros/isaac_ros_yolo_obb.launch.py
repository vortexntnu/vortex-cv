# SPDX-License-Identifier: MIT

"""Isaac ROS YOLO-OBB TensorRT inference pipeline.

Encoder pipeline (all inlined, no sub-launch):
  ImageFormatConverter → Resize → ImageToTensor → Normalize
  → InterleavedToPlanar → Reshape → TensorRT → OBBDecoder

Launch args (shared interface with ultralytics/ultralytics_yolo_obb.launch.py):
  image_topic, camera_info_topic, image_width, image_height,
  model_file_path, detections_topic, annotated_image_topic, confidence_threshold

Isaac-ROS-specific:
  engine_file_path
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

_TENSOR_PUB = '/yolo_obb/tensor_pub'
_TENSOR_SUB = '/yolo_obb/tensor_sub'
_CONVERTED = '/yolo_obb/internal/converted'
_RESIZE_IMAGE = '/yolo_obb/internal/resize/image'
_IMAGE_TENSOR = '/yolo_obb/internal/image_tensor'
_NORMALIZED = '/yolo_obb/internal/normalized'
_PLANAR = '/yolo_obb/internal/planar'


def _launch_setup(context, *args, **kwargs):
    pkg_dir = get_package_share_directory('perception_setup')
    with open(os.path.join(pkg_dir, 'config', 'yolo', 'yolo_obb.yaml')) as f:
        cfg = yaml.safe_load(f)

    image_topic = LaunchConfiguration('image_topic').perform(context)
    camera_info_topic = LaunchConfiguration('camera_info_topic').perform(context)
    image_width = int(LaunchConfiguration('image_width').perform(context))
    image_height = int(LaunchConfiguration('image_height').perform(context))
    model_file_path = LaunchConfiguration('model_file_path').perform(context)
    engine_file_path = LaunchConfiguration('engine_file_path').perform(context)
    detections_topic = LaunchConfiguration('detections_topic').perform(context)
    annotated_image_topic = LaunchConfiguration('annotated_image_topic').perform(context)
    confidence_threshold = float(LaunchConfiguration('confidence_threshold').perform(context))
    visualize = LaunchConfiguration('visualize').perform(context).lower() == 'true'

    image_format_converter = ComposableNode(
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
        name='image_format_converter',
        parameters=[{
            'encoding_desired': 'rgb8',
            'image_width': image_width,
            'image_height': image_height,
            'input_qos': 'sensor_data',
        }],
        remappings=[
            ('image_raw', image_topic),
            ('image', _CONVERTED),
        ],
    )

    resize_node = ComposableNode(
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ResizeNode',
        name='resize_node',
        parameters=[{
            'output_width': cfg['network_image_width'],
            'output_height': cfg['network_image_height'],
            'keep_aspect_ratio': False,
            'encoding_desired': 'rgb8',
        }],
        remappings=[
            ('/image', _CONVERTED),
            ('/resize/image', _RESIZE_IMAGE),
            ('/camera_info', camera_info_topic),
        ],
    )

    image_to_tensor = ComposableNode(
        package='isaac_ros_tensor_proc',
        plugin='nvidia::isaac_ros::dnn_inference::ImageToTensorNode',
        name='image_to_tensor',
        parameters=[{'tensor_name': 'image', 'scale': True}],
        remappings=[
            ('image', _RESIZE_IMAGE),
            ('tensor', _IMAGE_TENSOR),
        ],
    )

    normalize_node = ComposableNode(
        package='isaac_ros_tensor_proc',
        plugin='nvidia::isaac_ros::dnn_inference::ImageTensorNormalizeNode',
        name='normalize_node',
        parameters=[{
            'mean': cfg['image_mean'],
            'stddev': cfg['image_stddev'],
            'input_tensor_name': 'image',
            'output_tensor_name': 'image',
        }],
        remappings=[
            ('tensor', _IMAGE_TENSOR),
            ('normalized_tensor', _NORMALIZED),
        ],
    )

    interleaved_to_planar = ComposableNode(
        package='isaac_ros_tensor_proc',
        plugin='nvidia::isaac_ros::dnn_inference::InterleavedToPlanarNode',
        name='interleaved_to_planar',
        parameters=[{
            'input_tensor_shape': [cfg['network_image_height'], cfg['network_image_width'], 3],
            'num_blocks': 40,
        }],
        remappings=[
            ('interleaved_tensor', _NORMALIZED),
            ('planar_tensor', _PLANAR),
        ],
    )

    reshape_node = ComposableNode(
        package='isaac_ros_tensor_proc',
        plugin='nvidia::isaac_ros::dnn_inference::ReshapeNode',
        name='reshape_node',
        parameters=[{
            'input_tensor_shape': [3, cfg['network_image_height'], cfg['network_image_width']],
            'output_tensor_shape': [1, 3, cfg['network_image_height'], cfg['network_image_width']],
            'output_tensor_name': 'input_tensor',
            'num_blocks': 40,
        }],
        remappings=[
            ('tensor', _PLANAR),
            ('reshaped_tensor', _TENSOR_PUB),
        ],
    )

    tensor_rt_node = ComposableNode(
        name='tensor_rt',
        package='isaac_ros_tensor_rt',
        plugin='nvidia::isaac_ros::dnn_inference::TensorRTNode',
        parameters=[{
            'model_file_path': model_file_path,
            'engine_file_path': engine_file_path,
            'output_binding_names': cfg['output_binding_names'],
            'output_tensor_names': cfg['output_tensor_names'],
            'input_tensor_names': cfg['input_tensor_names'],
            'input_binding_names': cfg['input_binding_names'],
            'verbose': False,
            'force_engine_update': False,
        }],
        remappings=[
            ('tensor_pub', _TENSOR_PUB),
            ('tensor_sub', _TENSOR_SUB),
        ],
    )

    decoder_node = ComposableNode(
        name='yolo_obb_decoder',
        package='isaac_ros_yolov26_obb',
        plugin='nvidia::isaac_ros::yolov26_obb::YoloV26OBBDecoderNode',
        parameters=[{
            'tensor_input_topic': _TENSOR_SUB,
            'confidence_threshold': confidence_threshold,
            'num_detections': int(cfg['num_detections']),
            'detections_topic': detections_topic,
        }],
    )

    container = ComposableNodeContainer(
        name='yolo_obb_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            image_format_converter,
            resize_node,
            image_to_tensor,
            normalize_node,
            interleaved_to_planar,
            reshape_node,
            tensor_rt_node,
            decoder_node,
        ],
        output='screen',
        arguments=['--ros-args', '--log-level', 'INFO'],
    )

    visualizer = Node(
        package='isaac_ros_yolov26_obb',
        executable='isaac_ros_yolov26_obb_visualizer.py',
        name='yolo_obb_visualizer',
        parameters=[{
            'detections_topic': detections_topic,
            'image_topic': _RESIZE_IMAGE,
            'output_image_topic': annotated_image_topic,
            'class_names_yaml': str({0: 'valve'}),
        }],
    )

    actions = [container]
    if visualize:
        actions.append(visualizer)
    return actions


def generate_launch_description():
    pkg_dir = get_package_share_directory('perception_setup')
    models_dir = os.path.join(pkg_dir, 'models')

    return LaunchDescription([
        DeclareLaunchArgument(
            'image_topic',
            default_value='/realsense_d555/color/image_rect',
        ),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='/realsense_d555/color/camera_info',
        ),
        DeclareLaunchArgument('image_width', default_value='896'),
        DeclareLaunchArgument('image_height', default_value='504'),
        DeclareLaunchArgument(
            'model_file_path',
            default_value=os.path.join(models_dir, 'obb_best.onnx'),
        ),
        DeclareLaunchArgument(
            'engine_file_path',
            default_value=os.path.join(models_dir, 'obb_best.engine'),
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
            'visualize',
            default_value='true',
            description='Launch the OBB visualizer node to publish annotated images',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
