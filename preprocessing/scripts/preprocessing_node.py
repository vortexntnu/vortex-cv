#!/usr/bin/env python

import rospy

# msg types
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# classes
from confidence_mapping import ConfidenceMapping

class PreprocessingNode():
    """
    Class to handle operations related to the preprocessing node. \n
    This includes:
        - Confidence map --> masked confidence map
        - and so on
    """
    def __init__(self):
        rospy.init_node('preprocessing_node')
        rospy.Subscriber('/zed2/zed_node/confidence/confidence_map', Image, self.confidence_cb)

        self.maskedMapImagePub = rospy.Publisher('/cv/preprocessing/confidence_map_masked', Image, queue_size= 1)

        self.bridge = CvBridge()
        self.confMap = ConfidenceMapping()

    def confidence_cb(self, msg):
        """
        Gets a confidence map from camera through subscription
        and creates a mask where confidence above a threshold is set to 1 and below is set to 0.
        The masked confidence map is the same size as the original but includes only 0 and 1 as unique values.

        Args:
            msg: confidence map message from camera. Type: Image message.
        """
        # Bridge image data from Image to cv_image data
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

        # Make the masked map and store it in a class variable
        self.maskedMap, masked_as_cv_image = self.confMap.add_mask(self.cv_image, 50)

        # Bridge image data from cv_image to Image data
        # try:
        #     self.image = self.bridge.cv2_to_imgmsg(masked_as_cv_image, "passthrough")
        # except CvBridgeError as e:
        #     rospy.logerr("CvBridge Error: {0}".format(e))

        # masked_image = Image()
        # masked_image.header = msg.header
        # masked_image.height = msg.height
        # masked_image.width = msg.width
        # masked_image.encoding = msg.encoding
        # masked_image.is_bigendian = msg.is_bigendian
        # masked_image.step = msg.step
        # masked_image.data = self.image.data
        # self.maskedMapImagePub.publish(masked_image)


        

if __name__ == '__main__':
    node = PreprocessingNode()

    while not rospy.is_shutdown():
        rospy.spin()
