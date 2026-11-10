from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    yolo_params = os.path.join(
                get_package_share_directory("yolo_segmentation"),
                "config",
                "yolo_segmentation_params.yaml",
            )
    
    yolo = Node(
        package='yolo_segmentation',
        executable="yolo_seg_node",
        output = 'screen',
        parameters=[yolo_params],
    )

    path_marker = Node(
        package='path_marker_2d',
        executable= 'path_marker_2d_node',
        output = 'screen',
    )

    return LaunchDescription([yolo, path_marker])

