#!/usr/bin/python3

import rospy
import numpy as np
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion

"""

Simply publish Pose[] to \lidar_clusters.

"""


class Clusters:
    def __init__(self):
        rospy.init_node("test_node")
        self.pub = rospy.Publisher("/lidar/clusters", PoseArray, queue_size=10)

        # rospy.init_node()
        self.rate = rospy.Rate(10)

        self.x0 = 0
        self.y0 = 0

        self.x1 = 2
        self.y1 = 5

        self.x2 = -7
        self.y2 = 2

    def spin(self):
        while not rospy.is_shutdown():
            self.publish_pos_from_single_source()
            #self.publish_pos_from_multiple_sources()
            self.rate.sleep()

    def publish_pos_from_single_source(self):

        posearray = PoseArray()

        now = rospy.get_rostime()
        posearray.header.stamp.secs = now.secs


        self.x0 += 0.01
        self.y0 += 0.01

        p = Pose()
        p.position = Point(self.x0, self.y0, 0)

        posearray.poses.append(p)

        self.pub.publish(posearray)

    def publish_pos_from_multiple_sources(self):

        posearray = PoseArray()

        now = rospy.get_rostime()
        posearray.header.stamp.secs = now.secs

        self.x0 += 0.1
        self.y0 += 0.1
        p = Pose()
        p.position = Point(self.x0, self.y0, 0)
        posearray.poses.append(p)

        self.x1 -= 0.1
        self.y1 += 0.00
        p = Pose()
        p.position = Point(self.x1, self.y1, 0)
        posearray.poses.append(p)

        self.x2 += 0.15
        self.y2 += 0.2
        p = Pose()
        p.position = Point(self.x2, self.y2, 0)
        posearray.poses.append(p)

        self.pub.publish(posearray)


if __name__ == "__main__":
    try:
        generate_cluster = Clusters()
        generate_cluster.spin()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
