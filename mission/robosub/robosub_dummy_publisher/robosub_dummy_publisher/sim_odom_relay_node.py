"""Simulator odometry as the state estimator gives it on the vehicle.

The simulator publishes odometry in world_ned with child nautilus/odom and no
TF. On the vehicle the estimator gives <drone>/odom -> <drone>/base_link,
as a message and as TF. This node republishes the simulator's odometry that
way (odom_out, TF), so landmark_slam and the behavior tree run in the
simulator as on the vehicle. With odom_in a drifting odometry (the drift
injector), pose_out also gives its pose for the controller.
"""

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster


class SimOdomRelayNode(Node):
    def __init__(self):
        super().__init__("sim_odom_relay_node")
        self.declare_parameter("odom_in", "odom")
        self.declare_parameter("odom_out", "odom_nav")
        self.declare_parameter("pose_out", "")
        self.declare_parameter("odom_frame", "nautilus/odom")
        self.declare_parameter("base_frame", "nautilus/base_link")
        self._odom_frame = self.get_parameter("odom_frame").value
        self._base_frame = self.get_parameter("base_frame").value
        self._tf = TransformBroadcaster(self)
        self._odom_pub = self.create_publisher(
            Odometry, self.get_parameter("odom_out").value, qos_profile_sensor_data
        )
        pose_out = self.get_parameter("pose_out").value
        self._pose_pub = (
            self.create_publisher(
                PoseWithCovarianceStamped, pose_out, qos_profile_sensor_data
            )
            if pose_out
            else None
        )
        self.create_subscription(
            Odometry,
            self.get_parameter("odom_in").value,
            self._on_odom,
            qos_profile_sensor_data,
        )

    def _on_odom(self, msg: Odometry):
        msg.header.frame_id = self._odom_frame
        msg.child_frame_id = self._base_frame
        self._odom_pub.publish(msg)

        tf = TransformStamped()
        tf.header = msg.header
        tf.child_frame_id = self._base_frame
        p = msg.pose.pose.position
        tf.transform.translation.x = p.x
        tf.transform.translation.y = p.y
        tf.transform.translation.z = p.z
        tf.transform.rotation = msg.pose.pose.orientation
        self._tf.sendTransform(tf)

        if self._pose_pub:
            pose = PoseWithCovarianceStamped()
            pose.header = msg.header
            pose.pose = msg.pose
            self._pose_pub.publish(pose)


def main():
    rclpy.init()
    node = SimOdomRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
