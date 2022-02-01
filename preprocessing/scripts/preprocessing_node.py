#!/usr/bin/env python

import rospy

# msg types
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# classes
from confidence_masking import ConfidenceMasking


class PreprocessingNode():
    rospy.init_node('preprocessing_node')
    def __init__(self):
        rospy.Subscriber('/zed2/zed_node/confidence/confidence_map', Image, self.confidence_cb)
        self.bridge = CvBridge()
        self.confMask = ConfidenceMasking()

    def confidence_cb(self, msg):
        # Bridge image data from Image to cv_image data
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

        maskedMap = self.confMask.add_mask(self.cv_image, 50)
        rospy.loginfo(maskedMap)

if __name__ == '__main__':
    node = PreprocessingNode()

    while not rospy.is_shutdown():
        rospy.spin()
