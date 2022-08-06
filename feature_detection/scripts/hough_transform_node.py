#!/usr/bin/env python

# import debugpy
# print("Waiting for VSCode debugger...")
# debugpy.listen(5678)
# debugpy.wait_for_client()

import rospy
import rospkg

from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import cv2
import math
from timeit import default_timer as timer
import traceback

import copy

from Hough_Transform_orientation_based import HoughMajingo_ob

class HoughTransformNode():

    def __init__(self, image_topic):
        rospy.init_node('hough_transform_node')
        self.bridge = CvBridge()
        self.rospack = rospkg.RosPack()
        self.ros_rate = rospy.Rate(5.0)

        self.zedSub                 = rospy.Subscriber(image_topic, Image, self.camera_callback)
        
        self.hough_imgPub = rospy.Publisher('/feature_detection/hough/bbox_image', Image, queue_size= 1)
        self.edgesPub     = rospy.Publisher('/feature_detection/hough/edges_image', Image, queue_size= 1)
        
        # Canny params
        self.canny_threshold1 = 80 # 100
        self.canny_threshold2 = 150 # 200
        self.canny_aperture = 3
 
        # First initialization of image shape
        first_image_msg = rospy.wait_for_message(image_topic, Image)
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(first_image_msg, "passthrough")
            self.image_shape = self.cv_image.shape
        except CvBridgeError, e:
            self.image_shape = (720, 1280, 4)

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
                    cv_img_cp = copy.deepcopy(self.cv_image)
                    cropped_image = cv_img_cp[5:720, 0:1280]

                    bb_arr, center, hough_img, edges = HoughMajingo_ob.main(cropped_image, self.canny_threshold1, self.canny_threshold2)   # self.canny_threshold1, self.canny_threshold2, self.cv_image
                    self.cv_image_publisher(self.hough_imgPub, hough_img) 
                    self.cv_image_publisher(self.edgesPub, edges, '8UC1')
                
                except Exception, e:
                    rospy.logwarn(traceback.format_exc())
                    rospy.logwarn(rospy.get_rostime())

    def camera_callback(self, img_msg):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(img_msg, "passthrough")
        except CvBridgeError, e:
            rospy.logerr("CvBridge Error: {0}".format(e))

if __name__ == '__main__':
    try:
        hough_transform_node = HoughTransformNode(image_topic='/zed2/zed_node/rgb/image_rect_color')  # /cv/image_preprocessing/CLAHE  /zed2/zed_node/rgb/image_rect_color /cv/preprocessing/image_rect_color_filtered
        # rospy.spin()
        hough_transform_node.spin()

    except rospy.ROSInterruptException:
        pass