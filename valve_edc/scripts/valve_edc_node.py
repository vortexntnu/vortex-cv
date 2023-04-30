#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image

import dynamic_reconfigure.client

from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv
import numpy as np
import traceback


class CVTemplateNode():
    """Handles tasks related to feature detection
    """

    def __init__(self, image_topic):
        rospy.init_node('valve_edc_node')

        self.ros_rate = rospy.Rate(60.0)

        self.imgSub = rospy.Subscriber(image_topic, Image, self.img_callback)
        self.imgPub = rospy.Publisher('/cv_test/test', Image, queue_size=1)

        self.bridge = CvBridge()

        # test params
        self.test_param = 100
        self.test_enum = 3
        self.min_max_params = [0, 179, 0, 255, 0, 255]

        # First initialization of image shape
        first_image_msg = rospy.wait_for_message(image_topic, Image)
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(first_image_msg,
                                                      "passthrough")
            self.image_shape = self.cv_image.shape
        except CvBridgeError:
            raise (
                "CV Bridge was not successful in converting the ROS image...")

        self.dynam_client = dynamic_reconfigure.client.Client(
            "/cv_cfg/valve_edc_cfg",
            config_callback=self.dynam_reconfigure_callback)

    def cv_image_publisher(self, publisher, image, msg_encoding="bgra8"):
        """
        Takes a cv::Mat image object, converts it into a ROS Image message type, and publishes it using the specified publisher.
        """
        msgified_img = self.bridge.cv2_to_imgmsg(image, encoding=msg_encoding)
        publisher.publish(msgified_img)

    def spin(self):
        while not rospy.is_shutdown():
            if self.cv_image is not None:
                try:
                    gray = np.zeros(shape=self.cv_image.shape)
                    if self.cv_image.shape[2] == 3:
                        gray = cv.cvtColor(self.cv_image, cv.COLOR_BGR2GRAY)
                    else:
                        gray = cv.cvtColor(self.cv_image, cv.COLOR_BGRA2GRAY)

                    self.cv_image_publisher(self.imgPub,
                                            gray,
                                            msg_encoding="8UC1")

                except Exception:
                    rospy.logwarn(traceback.format_exc())
                    rospy.logwarn(rospy.get_rostime())

            self.ros_rate.sleep()

    def img_callback(self, img_msg):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(img_msg, "passthrough")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def dynam_reconfigure_callback(self, config):
        # Slider Example
        self.test_param = config.test_slider1

        # Enum example
        self.test_enum = config.test_enum

        # Min Max slider examples
        self.min_max_params[0] = config.test_min1
        self.min_max_params[1] = config.test_max1
        self.min_max_params[2] = config.test_min2
        self.min_max_params[3] = config.test_max2
        self.min_max_params[4] = config.test_min3
        self.min_max_params[5] = config.test_max3


if __name__ == '__main__':
    try:
        valve_edc_node = CVTemplateNode(
            image_topic='/zed2/zed_node/rgb_raw/image_raw_color')
        # rospy.spin()
        valve_edc_node.spin()

    except rospy.ROSInterruptException:
        pass
