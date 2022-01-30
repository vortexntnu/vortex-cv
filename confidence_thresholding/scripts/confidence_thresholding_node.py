#!/usr/bin/env python

import rospy

# msg types
from sensor_msgs.msg import Image


class ConfidenceThresholdingNode():
    rospy.init_node('confidence_thresholding_node')
    def __init__(self) -> None:
        rospy.Subscriber('/zed2/zed_node/confidence/confidence_map', Image, self.confidence_cb)

    def confidence_cb(self, msg):
        pass

if __name__ == '__main__':
    node = ConfidenceThresholdingNode()

    while not rospy.is_shutdown():
        rospy.spin()
