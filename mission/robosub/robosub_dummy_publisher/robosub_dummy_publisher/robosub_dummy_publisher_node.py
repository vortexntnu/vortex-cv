"""Publishes dummy vortex_msgs/LandmarkArray detections for RoboSub course elements.

For exercising landmark_server and mission logic without a running
perception stack or simulator. See course_layout.py for where the landmark
positions and role draws come from.
"""

import math
import random

import rclpy
from geometry_msgs.msg import PoseWithCovariance
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from vortex_msgs.msg import Landmark, LandmarkArray

from robosub_dummy_publisher.course_layout import TASKS, draw_role_picks


class RobosubDummyPublisherNode(Node):
    def __init__(self):
        super().__init__("robosub_dummy_publisher_node")

        # A number or a string; "-p seed:=7" gives an integer.
        self.declare_parameter("seed", "", ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter("tasks", list(TASKS.keys()))
        self.declare_parameter("rate", 2.0)
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("position_noise_std", 0.0)
        self.declare_parameter("topic", "landmarks")
        # Field of view: when enabled only landmarks the cameras could see from
        # the vehicle pose on `odom_topic` are published. Landmark positions
        # are then assumed to be in the frame of that odometry.
        self.declare_parameter("use_field_of_view", False)
        self.declare_parameter("odom_topic", "odom")
        self.declare_parameter("front_range_m", 8.0)
        self.declare_parameter("front_half_fov_deg", 45.0)
        self.declare_parameter("down_radius_m", 1.5)
        self.declare_parameter("down_max_altitude_m", 3.0)
        # Rotation variance >= 1000 means "no orientation".
        self.declare_parameter("no_orientation_variance", 1000.0)

        seed = self.get_parameter("seed").value
        task_names = self.get_parameter("tasks").value
        rate = self.get_parameter("rate").value
        self._frame_id = self.get_parameter("frame_id").value
        self._noise_std = self.get_parameter("position_noise_std").value
        topic = self.get_parameter("topic").value
        self._use_fov = self.get_parameter("use_field_of_view").value
        self._front_range = self.get_parameter("front_range_m").value
        self._front_half_fov = math.radians(
            self.get_parameter("front_half_fov_deg").value
        )
        self._down_radius = self.get_parameter("down_radius_m").value
        self._down_max_altitude = self.get_parameter("down_max_altitude_m").value
        self._no_ori_variance = self.get_parameter("no_orientation_variance").value
        self._vehicle = None  # (x, y, z, yaw)
        if self._use_fov:
            self.create_subscription(
                Odometry,
                self.get_parameter("odom_topic").value,
                self._on_odom,
                qos_profile_sensor_data,
            )

        unknown = [name for name in task_names if name not in TASKS]
        if unknown:
            self.get_logger().warn(
                f"Ignoring unknown task(s) {unknown}; available: {list(TASKS)}"
            )

        role_picks, resolved_seed = draw_role_picks(seed)
        self.get_logger().info(
            f"robosub_dummy_publisher seed {resolved_seed} "
            f"(reproduce with seed:={resolved_seed})"
        )
        if not role_picks:
            self.get_logger().warn(
                "stonefish_sim's role-image manifest wasn't found; "
                "gate/bin landmarks will use a fixed default role."
            )

        self._landmarks = []
        for name in task_names:
            task = TASKS.get(name)
            if task is None:
                continue
            for landmark in task.landmarks(role_picks):
                x = task.base_pose[0] + landmark.offset[0]
                y = task.base_pose[1] + landmark.offset[1]
                z = task.base_pose[2] + landmark.offset[2]
                self._landmarks.append((landmark, (x, y, z)))
                self.get_logger().info(
                    f"  {landmark.label:24s} type={landmark.landmark_type} "
                    f"subtype={landmark.landmark_subtype} "
                    f"pose=({x:.2f}, {y:.2f}, {z:.2f})"
                )

        self._publisher = self.create_publisher(LandmarkArray, topic, 10)
        self._timer = self.create_timer(1.0 / rate, self._publish)

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        self._vehicle = (p.x, p.y, p.z, yaw)

    def _visible(self, landmark, x, y, z) -> bool:
        """Whether the camera that sees this landmark could see it now."""
        if not self._use_fov:
            return True
        if self._vehicle is None:
            return False
        vx, vy, vz, yaw = self._vehicle
        dx, dy, dz = x - vx, y - vy, z - vz
        if landmark.camera == "down":
            # Below the vehicle, close enough in the horizontal plane (NED:
            # larger z is deeper).
            return (
                math.hypot(dx, dy) <= self._down_radius
                and 0.0 < dz <= self._down_max_altitude
            )
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)
        if distance > self._front_range or distance < 0.3:
            return False
        bearing = math.atan2(dy, dx) - yaw
        bearing = math.atan2(math.sin(bearing), math.cos(bearing))
        return abs(bearing) <= self._front_half_fov

    def _publish(self):
        stamp = self.get_clock().now().to_msg()
        msg = LandmarkArray()
        msg.header.stamp = stamp
        msg.header.frame_id = self._frame_id

        for i, (landmark, (x, y, z)) in enumerate(self._landmarks):
            if not self._visible(landmark, x, y, z):
                continue
            if self._noise_std > 0.0:
                x += random.gauss(0.0, self._noise_std)
                y += random.gauss(0.0, self._noise_std)
                z += random.gauss(0.0, self._noise_std)

            entry = Landmark()
            entry.header.stamp = stamp
            entry.header.frame_id = self._frame_id
            entry.id = i
            entry.type.value = landmark.landmark_type
            entry.subtype.value = landmark.landmark_subtype
            entry.pose = self._pose(x, y, z)
            msg.landmarks.append(entry)

        self._publisher.publish(msg)

    def _pose(self, x, y, z) -> PoseWithCovariance:
        pose = PoseWithCovariance()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0
        variance = self._noise_std**2 if self._noise_std > 0.0 else 1e-4
        for i in (0, 7, 14):
            pose.covariance[i] = variance
        # Position only: the orientation is a placeholder (identity) and the
        # rotation variance says so.
        for i in (21, 28, 35):
            pose.covariance[i] = self._no_ori_variance
        return pose


def main(args=None):
    rclpy.init(args=args)
    node = RobosubDummyPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
