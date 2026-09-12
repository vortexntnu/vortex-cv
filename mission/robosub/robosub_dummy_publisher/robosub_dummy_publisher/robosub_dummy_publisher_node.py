"""Publishes dummy vortex_msgs/LandmarkArray detections for RoboSub course elements.

For exercising landmark_server and mission logic without a running
perception stack or simulator. See course_layout.py for where the landmark
positions and role draws come from.
"""

import random

import rclpy
from geometry_msgs.msg import PoseWithCovariance
from rclpy.node import Node
from vortex_msgs.msg import Landmark, LandmarkArray

from robosub_dummy_publisher.course_layout import TASKS, draw_role_picks


class RobosubDummyPublisherNode(Node):
    def __init__(self):
        super().__init__("robosub_dummy_publisher_node")

        self.declare_parameter("seed", "")
        self.declare_parameter("tasks", list(TASKS.keys()))
        self.declare_parameter("rate", 2.0)
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("position_noise_std", 0.0)
        self.declare_parameter("topic", "landmarks")

        seed = self.get_parameter("seed").value
        task_names = self.get_parameter("tasks").value
        rate = self.get_parameter("rate").value
        self._frame_id = self.get_parameter("frame_id").value
        self._noise_std = self.get_parameter("position_noise_std").value
        topic = self.get_parameter("topic").value

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

    def _publish(self):
        stamp = self.get_clock().now().to_msg()
        msg = LandmarkArray()
        msg.header.stamp = stamp
        msg.header.frame_id = self._frame_id

        for i, (landmark, (x, y, z)) in enumerate(self._landmarks):
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
