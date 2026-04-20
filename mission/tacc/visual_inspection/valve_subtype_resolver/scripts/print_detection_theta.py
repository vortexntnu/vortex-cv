#!/usr/bin/env python3
"""Prints class_id + OBB theta (radians) per detection from the YOLO topic."""
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from vision_msgs.msg import Detection2DArray


class PrintDetectionTheta(Node):
    def __init__(self):
        super().__init__('print_detection_theta')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(
            Detection2DArray,
            '/ultralytics_valve_detection/detections',
            self.cb,
            qos,
        )

    def cb(self, msg: Detection2DArray):
        if not msg.detections:
            return
        parts = []
        for d in msg.detections:
            cls = d.results[0].hypothesis.class_id if d.results else '?'
            parts.append(
                f'id={cls} theta={d.bbox.center.theta:+.4f} rad '
                f'(w={d.bbox.size_x:.1f} h={d.bbox.size_y:.1f})'
            )
        self.get_logger().info(' | '.join(parts))


def main():
    rclpy.init()
    rclpy.spin(PrintDetectionTheta())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
