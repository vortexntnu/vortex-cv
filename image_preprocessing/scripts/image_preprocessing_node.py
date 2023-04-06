#! /usr/bin/env python

# import debugpy
# print("Waiting for VSCode debugger...")
# debugpy.listen(5678)
# debugpy.wait_for_client()

import rospy
import sys

from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError
import dynamic_reconfigure.client

from ImagePreprocessing import ImagePreprocessing

import numpy as np
import cv2 as cv


class ImagePreprocessingNode():
    """Performs image preprocessing (duh...)
    """

    def __init__(self, image_topic):
        rospy.init_node('image_preprocessing_node')

        ns = "/" + (image_topic.split("/"))[1]

        self.zedSub = rospy.Subscriber(image_topic,
                                       Image,
                                       self.image_callback,
                                       queue_size=1)

        self.CLAHEPub = rospy.Publisher('/cv/image_preprocessing/CLAHE' + ns,
                                        Image,
                                        queue_size=1)
        self.single_CLAHEPub = rospy.Publisher(
            '/cv/image_preprocessing/CLAHE_single' + ns, Image, queue_size=1)
        self.GWPub = rospy.Publisher('/cv/image_preprocessing/GW' + ns,
                                     Image,
                                     queue_size=1)

        self.bridge = CvBridge()
        self.image_preprocessing = ImagePreprocessing(2, 8)

        # First initialization of image shape
        first_image_msg = rospy.wait_for_message(image_topic, Image)

        self.cv_image = self.bridge.imgmsg_to_cv2(first_image_msg,
                                                  "passthrough")
        self.image_shape = self.cv_image.shape

    def cv_image_publisher(self, publisher, image, msg_encoding="bgra8"):
        """
        Takes a cv::Mat image object, converts it into a ROS Image message type, and publishes it using the specified publisher.
        """
        msgified_img = self.bridge.cv2_to_imgmsg(image, encoding=msg_encoding)
        publisher.publish(msgified_img)

    def image_callback(self, img_msg):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(img_msg, "passthrough")
        except CvBridgeError, e:
            rospy.logerr("CvBridge Error: {0}".format(e))

        clahe_img = self.image_preprocessing.CLAHE(self.cv_image)
        clahe_img = clahe_img.round().astype(np.uint8)
        try:
            self.cv_image_publisher(self.CLAHEPub, clahe_img, "bgra8")
            _, _, R, _ = cv.split(self.cv_image)
        except Exception:
            self.cv_image_publisher(self.CLAHEPub, clahe_img, "bgr8")
            _, _, R = cv.split(self.cv_image)

        clahe_r_channel = self.image_preprocessing.CLAHE(R)
        self.cv_image_publisher(self.single_CLAHEPub, clahe_r_channel, "mono8")

        gw_img = self.image_preprocessing.gray_world(
            cv.cvtColor(self.cv_image, cv.COLOR_BGRA2BGR))
        self.cv_image_publisher(self.GWPub, gw_img, "bgr8")


if __name__ == '__main__':
    try:
        image_topic = sys.argv[1]
        image_preprocessing_node = ImagePreprocessingNode(image_topic)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
