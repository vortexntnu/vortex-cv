#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from cv_msgs.msg import OdometryArray

"""
Replushied tracked objects as a path or odometry messages, so that it can be vizualized in Rviz. 
"""


class OdometryToPath:
    def __init__(self):
        rospy.init_node("visualizer_node")

        self.path_publisher = rospy.Publisher("nav_msgs/Path", Path, queue_size=10)
        rospy.Subscriber(
            "/tracking/tracked_cv_object", Odometry, self.single_track_callback
        )
        self.path = Path()
        self.path.header.frame_id = "os_lidar"

        self.odom_punblisher = rospy.Publisher(
            "/tracking/viz_mul_tracked_cv_objects", Odometry, queue_size=10
        )
        rospy.Subscriber(
            "/tracking/mul_tracked_cv_objects", OdometryArray, self.mul_tracks_callback
        )

    def single_track_callback(self, odometry):

        pose_stamped = PoseStamped()
        pose_stamped.pose = odometry.pose.pose
        pose_stamped.header = odometry.header
        self.path.poses.append(pose_stamped)
        self.path_publisher.publish(self.path)

    def mul_tracks_callback(self, odometryArray):

        for odom in odometryArray.odometry_array:
            self.odom_punblisher.publish(odom)


if __name__ == "__main__":
    try:
        odometry_to_path = OdometryToPath()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
