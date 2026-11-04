"""Shows the detections on `landmarks` as markers for Foxglove/RViz.

Real detections are small red spheres, clutter (id >= 1000, see
false_positive_rate) small magenta cubes. Each lives 0.3 s, so missed
detections, occlusions and the field of view show as flicker next to the
steady map from landmark_server.
"""

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from vortex_msgs.msg import LandmarkArray

CLUTTER_ID = 1000


class DetectionsMarkersNode(Node):
    def __init__(self):
        super().__init__("detections_markers_node")
        self.declare_parameter("landmarks_topic", "landmarks")
        self.declare_parameter("markers_topic", "debug/raw_detections")
        self._pub = self.create_publisher(
            MarkerArray, self.get_parameter("markers_topic").value, 10
        )
        self.create_subscription(
            LandmarkArray,
            self.get_parameter("landmarks_topic").value,
            self._on_landmarks,
            10,
        )

    def _on_landmarks(self, msg: LandmarkArray):
        out = MarkerArray()
        for k, lm in enumerate(msg.landmarks):
            clutter = lm.id >= CLUTTER_ID
            m = Marker()
            m.header.frame_id = lm.header.frame_id or msg.header.frame_id
            m.header.stamp = msg.header.stamp
            m.ns = "raw_detections"
            m.id = k
            m.type = Marker.CUBE if clutter else Marker.SPHERE
            m.action = Marker.ADD
            m.pose = lm.pose.pose
            m.scale.x = m.scale.y = m.scale.z = 0.12
            m.color.r, m.color.g, m.color.b, m.color.a = (
                (1.0, 0.0, 1.0, 1.0) if clutter else (1.0, 0.15, 0.15, 1.0)
            )
            m.lifetime = Duration(sec=0, nanosec=300_000_000)
            out.markers.append(m)
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = DetectionsMarkersNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
