from auv_setup.launch_arg_common import (
    declare_drone_and_namespace_args,
    resolve_drone_and_namespace,
)
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    _, namespace = resolve_drone_and_namespace(context)

    node = Node(
        package='pipeline_end_detector',
        executable='pipeline_end_detector_node',
        name='pipeline_end_detector_node',
        namespace=namespace,
        parameters=[
            {
                # Number of consecutive detections required before triggering the service call
                'detection_threshold': 100000,
                # Delay (seconds) between the start_detection trigger and detection becoming
                # active. Lets the FSM enter pipeline following immediately while suppressing
                # end-of-pipeline detection for an initial settling window. 0 = activate now.
                'activation_delay_sec': 30.0,
                # Publish the live detection counter on topics.debug_counter (std_msgs/Int32)
                # for plotting, e.g. with rqt_plot or PlotJuggler.
                'debug': True,
                # Topic published by the end-of-pipeline classifier (std_msgs/UInt8)
                # data == 1 -> Class 1: end of pipeline | data == 0 -> Class 0: continue following
                'topics.detection': 'classification_output',
                # Service name of the pipeline inspection FSM trigger
                'topics.end_of_pipeline_service': 'pipeline_inspection_fsm/pipeline_finished',
                # Service name to activate detection on this node
                'topics.start_detection_service': 'pipeline_end_detector/start_detection',
                # Topic the live detection counter is published on when debug == True
                'topics.debug_counter': 'pipeline_end_detector/debug_counter',
            }
        ],
        output='screen',
    )

    return [node]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        declare_drone_and_namespace_args() + [OpaqueFunction(function=launch_setup)]
    )
