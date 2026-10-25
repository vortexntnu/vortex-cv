#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import PoseArray
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from tf2_ros import Buffer, TransformListener
from vision_msgs.msg import Detection2DArray
from vortex_utils_ros.qos_profiles import sensor_data_profile


class ThetaMonitor(Node):
    def __init__(self):
        super().__init__("theta_monitor")

        # TF lookup for depth-to-color extrinsic
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.extrinsic_printed = False
        self.create_timer(1.0, self.print_extrinsic)

        self.create_subscription(
            Detection2DArray,
            "/obb_detections_output",
            self.obb_cb,
            sensor_data_profile(10),
        )
        self.create_subscription(
            PoseArray, "/valve_poses", self.pose_cb, sensor_data_profile(10)
        )

    def print_extrinsic(self):
        if self.extrinsic_printed:
            return
        try:
            tf = self.tf_buffer.lookup_transform(
                "nautilus/front_camera_color_optical",
                "nautilus/front_camera_depth_optical",
                rclpy.time.Time(),
            )
            q = tf.transform.rotation
            t = tf.transform.translation
            r = Rotation.from_quat([q.x, q.y, q.z, q.w])
            rot = r.as_matrix()
            roll, pitch, yaw = r.as_euler("xyz", degrees=True)

            self.get_logger().info(
                f"\n{'=' * 60}\n"
                f"DEPTH-TO-COLOR EXTRINSIC (depth -> color)\n"
                f"  Translation: [{t.x:.6f}, {t.y:.6f}, {t.z:.6f}]\n"
                f"  RPY (xyz):   roll={roll:.4f}°  pitch={pitch:.4f}°  yaw={yaw:.4f}°\n"
                f"  Rotation matrix:\n"
                f"    [{rot[0, 0]:9.6f}  {rot[0, 1]:9.6f}  {rot[0, 2]:9.6f}]\n"
                f"    [{rot[1, 0]:9.6f}  {rot[1, 1]:9.6f}  {rot[1, 2]:9.6f}]\n"
                f"    [{rot[2, 0]:9.6f}  {rot[2, 1]:9.6f}  {rot[2, 2]:9.6f}]\n"
                f"\n"
                f"  R_depth_from_color (R^T, used in pose_estimator):\n"
                f"    [{rot[0, 0]:9.6f}  {rot[1, 0]:9.6f}  {rot[2, 0]:9.6f}]\n"
                f"    [{rot[0, 1]:9.6f}  {rot[1, 1]:9.6f}  {rot[2, 1]:9.6f}]\n"
                f"    [{rot[0, 2]:9.6f}  {rot[1, 2]:9.6f}  {rot[2, 2]:9.6f}]\n"
                f"{'=' * 60}"
            )
            self.extrinsic_printed = True
        except Exception as e:
            self.get_logger().warn(f"Waiting for TF: {e}")

    def obb_cb(self, msg: Detection2DArray):
        for i, det in enumerate(msg.detections):
            theta_rad = det.bbox.center.theta
            theta_deg = math.degrees(theta_rad)
            self.get_logger().info(
                f"[OBB det {i}] theta: {theta_rad:.4f} rad  ({theta_deg:.2f} deg)"
            )

    def pose_cb(self, msg: PoseArray):
        for i, pose in enumerate(msg.poses):
            q = pose.orientation
            r = Rotation.from_quat([q.x, q.y, q.z, q.w])
            roll, pitch, yaw = r.as_euler("xyz", degrees=True)
            self.get_logger().info(
                f"[POSE {i}] "
                f"roll: {roll:7.2f}°  pitch: {pitch:7.2f}°  yaw: {yaw:7.2f}°  "
                f"pos: ({pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f})"
            )


def main():
    rclpy.init()
    rclpy.spin(ThetaMonitor())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
