#!/usr/bin/env python3
"""Prints valve landmark yaw in the world (odom) frame.

Subscribes to /valve_landmarks_typed, transforms each landmark pose into the
world frame via TF2, and prints yaw in degrees.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

import tf2_ros
from tf2_geometry_msgs import do_transform_pose_stamped
from geometry_msgs.msg import PoseStamped
from vortex_msgs.msg import LandmarkArray


def quat_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class PrintValveYaw(Node):
    def __init__(self):
        super().__init__('print_valve_yaw')
        self.declare_parameter('drone', 'nautilus')
        self.declare_parameter('world_frame', 'odom')
        drone = self.get_parameter('drone').value
        world = self.get_parameter('world_frame').value
        self.world_frame = f'{drone}/{world}'

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(
            LandmarkArray, '/valve_landmarks_typed', self.cb, qos
        )

    def cb(self, msg: LandmarkArray):
        if not msg.landmarks:
            return
        parts = []
        for lm in msg.landmarks:
            ps = PoseStamped()
            ps.header = lm.header
            ps.pose = lm.pose.pose
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.world_frame,
                    lm.header.frame_id,
                    rclpy.time.Time(),
                )
                ps_w = do_transform_pose_stamped(ps, tf)
            except Exception as ex:
                self.get_logger().warn(f'TF lookup failed: {ex}', throttle_duration_sec=1.0)
                return
            yaw = quat_to_yaw(ps_w.pose.orientation)
            parts.append(f'id={lm.id} yaw_world={math.degrees(yaw):+7.2f}°')
        self.get_logger().info(' | '.join(parts))


def main():
    rclpy.init()
    rclpy.spin(PrintValveYaw())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
