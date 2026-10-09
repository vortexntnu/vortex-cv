"""Publishes dummy vortex_msgs/LandmarkArray detections for RoboSub course elements.

For exercising landmark_server and mission logic without a running
perception stack or simulator. See course_layout.py for where the landmark
positions and role draws come from.

With use_field_of_view the detections behave like a camera's: every error
effect is a [near, far] pair, interpolated by how badly the object is seen.
Up close (within near_range_m) and in the middle of the image (within
centre_half_fov_deg) the near values hold: accurate and consistent. Toward
front_range_m or the edge of the field of view the far values take over:
noise, misses, dropouts, outliers, clutter, phantoms and confused classes.
"""

import dataclasses
import math
import random

import rclpy
from geometry_msgs.msg import PoseWithCovariance
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from vortex_msgs.msg import Landmark, LandmarkArray

from robosub_dummy_publisher.course_layout import (
    CONFUSIONS,
    DECOYS,
    TASKS,
    draw_role_picks,
)


def _clamp01(v: float) -> float:
    return min(1.0, max(0.0, v))


class RobosubDummyPublisherNode(Node):
    def __init__(self):
        super().__init__("robosub_dummy_publisher_node")

        # A number or a string; "-p seed:=7" gives an integer.
        self.declare_parameter("seed", "", ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter("tasks", list(TASKS.keys()))
        self.declare_parameter("rate", 2.0)
        self.declare_parameter("frame_id", "world")
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
        # View quality: within near_range_m and centre_half_fov_deg of the
        # image centre an object is seen best (the near values below); it
        # gets worse linearly up to front_range_m and the edge of the view.
        self.declare_parameter("near_range_m", 2.0)
        self.declare_parameter("centre_half_fov_deg", 15.0)
        # Rotation variance >= 1000 means "no orientation".
        self.declare_parameter("no_orientation_variance", 1000.0)

        # Noise. Std [m] along the line of sight (range) and across it,
        # [base, per metre of distance]: a camera's range is much worse than
        # its bearing. Both grow by off_axis_noise_gain at the edge of the
        # view (lens distortion, a part of the object cut off). A range bias
        # as a fraction of the distance (+ = too far). position_noise_std:
        # isotropic, on top (the only noise without use_field_of_view).
        self.declare_parameter("position_noise_std", 0.0)
        self.declare_parameter("range_noise_along", [0.0, 0.0])
        self.declare_parameter("range_noise_across", [0.0, 0.0])
        self.declare_parameter("off_axis_noise_gain", 0.0)
        self.declare_parameter("range_bias_per_m", 0.0)
        # Yaw noise [deg] of a measured surface normal (torpedo board).
        self.declare_parameter("orientation_noise_deg", [0.0, 0.0])

        # [near, far] pairs: per landmark and frame unless stated otherwise.
        self.declare_parameter("detection_probability", [1.0, 1.0])
        # Occlusions: a landmark disappears for a while. Rate of new
        # occlusions per landmark [1/s], and their duration range [s].
        self.declare_parameter("dropout_rate_per_sec", [0.0, 0.0])
        self.declare_parameter("dropout_duration_sec", [1.0, 5.0])
        # A detection far off its landmark (bad depth, wrong association).
        self.declare_parameter("outlier_probability", [0.0, 0.0])
        self.declare_parameter("outlier_std_m", 1.0)
        # Clutter: spurious detections per frame (expected count, the mean
        # over the landmarks in view), each a copy of the class of a landmark
        # in view within false_positive_radius_m of it.
        self.declare_parameter("false_positive_rate", [0.0, 0.0])
        self.declare_parameter("false_positive_radius_m", 2.0)
        # Class confusion (course_layout.CONFUSIONS: white <-> red pipe, the
        # icons of one shape): per detection, and in episodes (a bad view
        # that lasts) at class_confusion_rate_per_sec at the far end (none up
        # close) for a duration [s].
        self.declare_parameter("class_confusion_probability", [0.0, 0.0])
        self.declare_parameter("class_confusion_rate_per_sec", 0.0)
        self.declare_parameter("class_confusion_duration_sec", [0.5, 3.0])
        # Phantoms: objects that are not there but are detected again and
        # again at one place (a reflection, a shadow, another prop), each a
        # copy of the class of a real landmark at a distance in the range
        # from it, detected with phantom_probability [near, far] when in
        # view: seen from afar, gone up close.
        self.declare_parameter("phantom_count", 0)
        self.declare_parameter("phantom_probability", [0.0, 0.3])
        self.declare_parameter("phantom_distance_m", [0.8, 2.5])
        self.declare_parameter("frame_drop_probability", 0.0)  # whole message
        # Decoys: other course objects taken for a class (the gate's posts
        # seen as slalom pipes, course_layout.DECOYS), each detected with this
        # probability per frame when in view. 0 = off.
        self.declare_parameter("decoy_probability", 0.0)
        # Loose objects (the jars and containers on the table) are moved to a
        # new spot within the radius of where they started, on average every
        # interval seconds each. 0 = they stay put.
        # Integers are accepted too ("-p movable_move_interval_sec:=10").
        number = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter("movable_move_interval_sec", 0.0, number)
        self.declare_parameter("movable_move_radius_m", 0.5, number)
        # Seed for the noise and the instability. -1 = a fresh draw each run.
        self.declare_parameter("noise_seed", -1)
        # A slow detector: each frame is published this long after the time
        # it is stamped with (the image time). [s], 0 = at once.
        self.declare_parameter("latency_sec", 0.0)

        def get(name):
            return self.get_parameter(name).value

        def pair(name):
            value = tuple(float(v) for v in get(name))
            if len(value) != 2:
                raise ValueError(f"{name}: give [near, far]")
            return value

        seed = get("seed")
        task_names = get("tasks")
        rate = get("rate")
        self._frame_id = get("frame_id")
        topic = get("topic")
        self._use_fov = get("use_field_of_view")
        self._front_range = get("front_range_m")
        self._front_half_fov = math.radians(get("front_half_fov_deg"))
        self._down_radius = get("down_radius_m")
        self._down_max_altitude = get("down_max_altitude_m")
        self._near_range = get("near_range_m")
        self._centre_half_fov = math.radians(get("centre_half_fov_deg"))
        self._no_ori_variance = get("no_orientation_variance")
        self._noise_std = get("position_noise_std")
        self._range_along = pair("range_noise_along")
        self._range_across = pair("range_noise_across")
        self._off_axis_gain = get("off_axis_noise_gain")
        self._range_bias = get("range_bias_per_m")
        self._ori_noise = tuple(math.radians(v) for v in pair("orientation_noise_deg"))
        self._p_detect = pair("detection_probability")
        self._dropout_rate = pair("dropout_rate_per_sec")
        self._dropout_duration = tuple(get("dropout_duration_sec"))
        self._p_outlier = pair("outlier_probability")
        self._outlier_std = get("outlier_std_m")
        self._fp_rate = pair("false_positive_rate")
        self._fp_radius = get("false_positive_radius_m")
        self._p_confusion = pair("class_confusion_probability")
        self._confusion_rate = get("class_confusion_rate_per_sec")
        self._confusion_duration = tuple(get("class_confusion_duration_sec"))
        phantom_count = get("phantom_count")
        self._p_phantom = pair("phantom_probability")
        phantom_distance = tuple(get("phantom_distance_m"))
        self._p_frame_drop = get("frame_drop_probability")
        self._p_decoy = get("decoy_probability")
        self._move_interval = float(get("movable_move_interval_sec"))
        self._move_radius = float(get("movable_move_radius_m"))
        noise_seed = get("noise_seed")
        self._rng = random.Random(None if noise_seed < 0 else noise_seed)
        self._period = 1.0 / rate
        self._vehicle = None  # (x, y, z, yaw)
        if self._use_fov:
            self.create_subscription(
                Odometry, get("odom_topic"), self._on_odom, qos_profile_sensor_data
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

        if self._p_decoy > 0.0:
            for landmark, pos in DECOYS:
                self._landmarks.append((landmark, pos))
                self.get_logger().info(
                    f"  {landmark.label:24s} decoy type={landmark.landmark_type} "
                    f"subtype={landmark.landmark_subtype} p={self._p_decoy} "
                    f"pose=({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})"
                )

        # Phantoms next to the real landmarks of the tasks (not the decoys).
        real = [(lm, pos) for lm, pos in self._landmarks if not lm.decoy]
        self._phantoms = set()
        for k in range(phantom_count if real else 0):
            landmark, (x, y, z) = self._rng.choice(real)
            r = self._rng.uniform(*phantom_distance)
            a = self._rng.uniform(-math.pi, math.pi)
            pos = (x + r * math.cos(a), y + r * math.sin(a), z)
            phantom = dataclasses.replace(
                landmark, label=f"phantom_{k}_{landmark.label}", movable=False
            )
            self._phantoms.add(len(self._landmarks))
            self._landmarks.append((phantom, pos))
            self.get_logger().info(
                f"  {phantom.label:24s} phantom type={phantom.landmark_type} "
                f"subtype={phantom.landmark_subtype} p={self._p_phantom} "
                f"pose=({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})"
            )

        self._occluded_until = [0.0] * len(self._landmarks)
        self._confused_until = [0.0] * len(self._landmarks)
        self._home = [pos for _, pos in self._landmarks]
        if self._move_interval > 0.0:
            n = sum(1 for lm, _ in self._landmarks if lm.movable)
            self.get_logger().info(
                f"{n} loose object(s) move every ~{self._move_interval} s "
                f"within {self._move_radius} m"
            )
        self.get_logger().info(
            f"detector: field of view {'on' if self._use_fov else 'off'} "
            f"(range {self._front_range} m, half fov "
            f"{math.degrees(self._front_half_fov):.0f} deg, best within "
            f"{self._near_range} m and {math.degrees(self._centre_half_fov):.0f} "
            f"deg); [near, far]: detect p {self._p_detect}, dropouts "
            f"{self._dropout_rate}/s, outliers p {self._p_outlier}, clutter "
            f"{self._fp_rate}/frame, confusion p {self._p_confusion}, "
            f"{len(self._phantoms)} phantom(s) p {self._p_phantom}; noise along "
            f"{self._range_along}, across {self._range_across} (base + per_m * d), "
            f"+{self._noise_std} m; noise_seed {noise_seed}"
        )
        if not self._use_fov:
            self.get_logger().info(
                "no field of view: the near values hold everywhere, range noise is off"
            )

        self._publisher = self.create_publisher(LandmarkArray, topic, 10)
        self._timer = self.create_timer(1.0 / rate, self._publish)
        self._latency = float(get("latency_sec"))
        self._delayed = []  # (publish time, message), oldest first
        if self._latency > 0.0:
            self.create_timer(0.02, self._flush_delayed)
            self.get_logger().info(
                f"detections published {self._latency:.2f} s after their stamp"
            )

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        self._vehicle = (p.x, p.y, p.z, yaw)

    def _bearing(self, x, y) -> float:
        """Horizontal angle of the point from the camera's axis [rad]."""
        vx, vy, _, yaw = self._vehicle
        b = math.atan2(y - vy, x - vx) - yaw
        return abs(math.atan2(math.sin(b), math.cos(b)))

    def _visible(self, landmark, x, y, z) -> bool:
        """Whether the camera that sees this landmark could see it now."""
        if not self._use_fov:
            return True
        if self._vehicle is None:
            return False
        vx, vy, vz, _ = self._vehicle
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
        return self._bearing(x, y) <= self._front_half_fov

    def _view(self, landmark, x, y, z):
        """(badness, off-axis part), both 0 (best) .. 1 (worst).

        Badness is the worse of the distance part (near_range_m ..
        front_range_m) and the off-axis part (centre_half_fov_deg .. the
        edge of the view). The down camera only has the distance part.
        """
        if not self._use_fov or self._vehicle is None:
            return 0.0, 0.0
        vx, vy, vz, _ = self._vehicle
        d = math.sqrt((x - vx) ** 2 + (y - vy) ** 2 + (z - vz) ** 2)
        if landmark.camera == "down":
            return _clamp01((d - 0.5) / max(1e-6, self._down_max_altitude - 0.5)), 0.0
        far = _clamp01(
            (d - self._near_range) / max(1e-6, self._front_range - self._near_range)
        )
        edge = _clamp01(
            (self._bearing(x, y) - self._centre_half_fov)
            / max(1e-6, self._front_half_fov - self._centre_half_fov)
        )
        return max(far, edge), edge

    @staticmethod
    def _lerp(near_far, b: float) -> float:
        near, far = near_far
        return near + (far - near) * b

    def _detected(self, i: int, now: float, b: float) -> bool:
        """Whether landmark i is detected in this frame (occlusion, misses)."""
        if now < self._occluded_until[i]:
            return False
        if self._rng.random() < self._lerp(self._dropout_rate, b) * self._period:
            self._occluded_until[i] = now + self._rng.uniform(*self._dropout_duration)
            return False
        landmark = self._landmarks[i][0]
        if landmark.decoy:
            p = self._p_decoy
        elif i in self._phantoms:
            p = self._lerp(self._p_phantom, b)
        else:
            p = self._lerp(self._p_detect, b)
        return self._rng.random() < p

    def _class_of(self, i: int, now: float, b: float):
        """The class landmark i is reported as in this frame."""
        landmark = self._landmarks[i][0]
        key = (landmark.landmark_type, landmark.landmark_subtype)
        others = CONFUSIONS.get(key)
        if not others:
            return key
        if self._rng.random() < self._confusion_rate * b * self._period:
            self._confused_until[i] = now + self._rng.uniform(*self._confusion_duration)
        if now < self._confused_until[i] or self._rng.random() < self._lerp(
            self._p_confusion, b
        ):
            return self._rng.choice(others)
        return key

    def _publish(self):
        now_time = self.get_clock().now()
        now = now_time.nanoseconds * 1e-9
        stamp = now_time.to_msg()
        self._move_loose_objects()

        visible = []  # (index, badness, off-axis part)
        for i, (landmark, (x, y, z)) in enumerate(self._landmarks):
            if self._visible(landmark, x, y, z):
                visible.append((i, *self._view(landmark, x, y, z)))
        # Advance the occlusions even when the frame is lost.
        detected = {i: self._detected(i, now, b) for i, b, _ in visible}
        if self._rng.random() < self._p_frame_drop:
            return

        msg = LandmarkArray()
        msg.header.stamp = stamp
        msg.header.frame_id = self._frame_id
        for i, b, edge in visible:
            if not detected[i]:
                continue
            landmark, (x, y, z) = self._landmarks[i]
            x, y, z = self._perturb(x, y, z, b, edge)
            msg.landmarks.append(
                self._entry(stamp, i, landmark, x, y, z, b, self._class_of(i, now, b))
            )

        # Clutter: copies of the class of a landmark in view, close to it.
        for i, b, edge in visible:
            rate = self._lerp(self._fp_rate, b) / len(visible)
            for k in range(self._poisson(rate)):
                landmark, (x, y, z) = self._landmarks[i]
                r = self._fp_radius * math.sqrt(self._rng.random())
                a = self._rng.uniform(-math.pi, math.pi)
                x, y, z = self._perturb(
                    x + r * math.cos(a), y + r * math.sin(a), z, b, edge
                )
                msg.landmarks.append(
                    self._entry(stamp, 1000 + 10 * i + k, landmark, x, y, z, b)
                )

        if self._latency > 0.0:
            self._delayed.append((now + self._latency, msg))
        else:
            self._publisher.publish(msg)

    def _flush_delayed(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        while self._delayed and self._delayed[0][0] <= now:
            self._publisher.publish(self._delayed.pop(0)[1])

    def _move_loose_objects(self):
        if self._move_interval <= 0.0:
            return
        for i, (landmark, _) in enumerate(self._landmarks):
            if not landmark.movable:
                continue
            if self._rng.random() >= self._period / self._move_interval:
                continue
            hx, hy, hz = self._home[i]
            r = self._move_radius * math.sqrt(self._rng.random())
            a = self._rng.uniform(-math.pi, math.pi)
            new = (hx + r * math.cos(a), hy + r * math.sin(a), hz)
            self._landmarks[i] = (landmark, new)
            self.get_logger().info(
                f"moved {landmark.label} to ({new[0]:.2f}, {new[1]:.2f}, {new[2]:.2f})"
            )

    def _perturb(self, x, y, z, b, edge):
        """Isotropic noise (or an outlier), then line-of-sight noise.

        Along and across the line of sight, growing with the distance and
        toward the edge of the view.
        """
        std = self._noise_std
        if self._rng.random() < self._lerp(self._p_outlier, b):
            std = self._outlier_std
        if std > 0.0:
            x += self._rng.gauss(0.0, std)
            y += self._rng.gauss(0.0, std)
            z += self._rng.gauss(0.0, std)
        if not self._use_fov or self._vehicle is None:
            return x, y, z
        vx, vy, vz, _ = self._vehicle
        d = math.sqrt((x - vx) ** 2 + (y - vy) ** 2 + (z - vz) ** 2)
        if d < 1e-6:
            return x, y, z
        u = ((x - vx) / d, (y - vy) / d, (z - vz) / d)
        # Across: one horizontal direction, and the one perpendicular to
        # both (a line of sight straight down has no horizontal normal).
        h = math.hypot(u[0], u[1])
        a1 = (-u[1] / h, u[0] / h, 0.0) if h > 1e-6 else (1.0, 0.0, 0.0)
        a2 = (
            u[1] * a1[2] - u[2] * a1[1],
            u[2] * a1[0] - u[0] * a1[2],
            u[0] * a1[1] - u[1] * a1[0],
        )
        gain = 1.0 + self._off_axis_gain * edge
        along = self._range_bias * d
        std_along = (self._range_along[0] + self._range_along[1] * d) * gain
        std_across = (self._range_across[0] + self._range_across[1] * d) * gain
        if std_along > 0.0:
            along += self._rng.gauss(0.0, std_along)
        c1 = self._rng.gauss(0.0, std_across) if std_across > 0.0 else 0.0
        c2 = self._rng.gauss(0.0, std_across) if std_across > 0.0 else 0.0
        return (
            x + along * u[0] + c1 * a1[0] + c2 * a2[0],
            y + along * u[1] + c1 * a1[1] + c2 * a2[1],
            z + along * u[2] + c1 * a1[2] + c2 * a2[2],
        )

    def _poisson(self, lam: float) -> int:
        # Knuth; lam is small (a few per frame at most).
        if lam <= 0.0:
            return 0
        limit, k, p = math.exp(-lam), 0, 1.0
        while True:
            p *= self._rng.random()
            if p <= limit:
                return k
            k += 1

    def _entry(self, stamp, i, landmark, x, y, z, b, key=None) -> Landmark:
        entry = Landmark()
        entry.header.stamp = stamp
        entry.header.frame_id = self._frame_id
        entry.id = i
        entry.type.value, entry.subtype.value = key or (
            landmark.landmark_type,
            landmark.landmark_subtype,
        )
        entry.pose = self._pose(x, y, z, landmark.normal_yaw, b)
        return entry

    def _pose(self, x, y, z, normal_yaw, b) -> PoseWithCovariance:
        pose = PoseWithCovariance()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        variance = self._noise_std**2 if self._noise_std > 0.0 else 1e-4
        for i in (0, 7, 14):
            pose.covariance[i] = variance
        if normal_yaw is None:
            # Position only: the orientation is a placeholder (identity) and
            # the rotation variance says so.
            pose.pose.orientation.w = 1.0
            for i in (21, 28, 35):
                pose.covariance[i] = self._no_ori_variance
            return pose
        # A measured surface normal: +X out of the front, yaw only.
        std = self._lerp(self._ori_noise, b)
        yaw = normal_yaw + (self._rng.gauss(0.0, std) if std > 0.0 else 0.0)
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        for i in (21, 28, 35):
            pose.covariance[i] = max(std, math.radians(1.0)) ** 2
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
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
