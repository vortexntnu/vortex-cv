#!/usr/bin/python3

import rospy
import numpy as np
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion

"""

Simply publish Pose[] to \lidar_clusters.

"""


class Clusters:
    def __init__(self):
        rospy.init_node("Test_tracker")
        self.pub = rospy.Publisher("/lidar/clusters", PoseArray, queue_size=10)

        # rospy.init_node()
        self.rate = rospy.Rate(10)

        self.x = 0
        self.y = 0

    def spin(self):
        while not rospy.is_shutdown():
            self.publish_poses()
            self.rate.sleep()

    def publish_poses(self):

        posearray = PoseArray()

        now = rospy.get_rostime()
        posearray.header.stamp.secs = now.secs

        for i in range(5):

            self.x += 0.01
            self.y += 0.01

            p = Pose()
            p.position = Point(self.x, self.y, 0)

            posearray.poses.append(p)

        self.pub.publish(posearray)


if __name__ == "__main__":
    try:
        generate_cluster = Clusters()
        generate_cluster.spin()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
