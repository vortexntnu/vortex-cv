#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox
from cv_msgs.msg import CenteroidArray, Centeroid

import dynamic_reconfigure.client

from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv
import numpy as np
import traceback

from copy import deepcopy

from position_estimator import PositionEstimator

class ValveEDC():
    """Handles tasks related to feature detection
    """

    def __init__(self, image_topic):
        rospy.init_node('valve_edc_node')

        self.ros_rate = rospy.Rate(10.0)

        self.imgSub = rospy.Subscriber(image_topic, Image, self.img_callback)
        self.bboxSub = rospy.Subscriber("/feature_detection/sift_detection_bbox", BoundingBoxes, self.bbox_cb)
        self.cntrSub = rospy.Subscriber("/feature_detection/sift_detection_centeroid", CenteroidArray, self.centeroid_cb)
        
        self.imgPubGray = rospy.Publisher('/valve/gray', Image, queue_size=1)
        self.imgPubCircle = rospy.Publisher('/valve/circle',
                                            Image,
                                            queue_size=1)

        self.bridge = CvBridge()

        # test params
        self.test_param1 = 100
        self.test_param2 = 100
        self.kernel = 5
        self.sigma = 1.0

        self.pos_est = PositionEstimator()

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
            "/valve_edc_cfg/cv_cfg",
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

                img = deepcopy(self.cv_image)
                try:
                    gray = np.zeros(shape=self.cv_image.shape)
                    if self.cv_image.shape[2] == 3:
                        gray = cv.cvtColor(self.cv_image, cv.COLOR_BGR2GRAY)
                    else:
                        gray = cv.cvtColor(self.cv_image, cv.COLOR_BGRA2GRAY)


                    rospy.loginfo(self.bbox)
                    rospy.loginfo(self.centeroid)
                    length_x_mtr, length_y_mtr, redefined_angle_x, redefined_angle_y = self.pos_est.main(self.bbox, 0.5)
                    rospy.loginfo(length_x_mtr)
                    rospy.loginfo(length_y_mtr)
                    rospy.loginfo(redefined_angle_x)
                    rospy.loginfo(redefined_angle_y)
                    
                    rospy.loginfo("\n")







                    self.cv_image_publisher(self.imgPubGray,
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

    def bbox_cb(self, msg):
        self.bbox = msg.bounding_boxes[0]

    def centeroid_cb(self, msg):
        self.centeroid = msg.centeroid[0]

    def dynam_reconfigure_callback(self, config):
        # Slider Example
        self.test_param1 = config.test_slider1
        self.test_param2 = config.test_slider2
        self.kernel = config.kernel
        self.sigma = config.sigma


if __name__ == '__main__':
    try:
        valve_edc_node = ValveEDC(image_topic='/image')
        valve_edc_node.spin()

    except rospy.ROSInterruptException:
        pass
