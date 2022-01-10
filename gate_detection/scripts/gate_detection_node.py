#!/usr/bin/env python

import rospy

from sensor_msgs.msg import Image
from std_msgs.msg import Float32

from cv_bridge import CvBridge, CvBridgeError
import dynamic_reconfigure.client

import numpy as np
import cv2
import copy
from timeit import default_timer as timer


class GateDetectionNode():
    """Handles tasks related to gate detection
    """

    def __init__(self):
        rospy.init_node('gate_detection_node')

        self.zedSub = rospy.Subscriber('/zed2/zed_node/rgb/image_rect_color', Image, self.zedSub_callback)
        
        self.linesPub = rospy.Publisher('/gate_detection/lines_image', Image, queue_size= 1)
        self.cannyPub = rospy.Publisher('/gate_detection/canny_image', Image, queue_size= 1)
        
        self.timerPub = rospy.Publisher('/gate_detection/timer', Float32, queue_size= 1)

        self.bridge = CvBridge()

        # Canny params
        self.canny_threshold1 = 100
        self.canny_threshold2 = 200
        self.canny_aperture = 3


        self.dynam_client = dynamic_reconfigure.client.Client("gate_detection_cfg", config_callback=self.dynam_reconfigure_callback)

    

    def lines_publisher(self, orig_img, edges_img):
        start = timer() # Start function timer.
        #------------------------------------------>

        orig_img_cp = copy.deepcopy(orig_img)

        lines = cv2.HoughLines(edges_img, 1, np.pi/180, 200)

        lines_matrix = []

        for line in lines:
            for rho, theta in line:
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a*rho
                y0 = b*rho
                x1 = int(x0 + 10000*(-b))
                y1 = int(y0 + 10000*(a))
                x2 = int(x0 - 10000*(-b))
                y2 = int(y0 - 10000*(a))

                lines_matrix.append([(x1,y1),(x2,y2)])
                cv2.line(orig_img_cp, (x1,y1), (x2,y2), (0,0,255), 2)
        # print(lines_matrix)

        ros_image = self.bridge.cv2_to_imgmsg(orig_img_cp, encoding="bgra8")
        self.linesPub.publish(ros_image)

        #------------------------------------------>
        end = timer() # Stop function timer.
        timediff = (end - start)

        fps = 1 / timediff # Take reciprocal of the timediff to get runs per second. 

        self.timerPub.publish(fps)


    def zedSub_callback(self, img_msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, "passthrough")
        except CvBridgeError, e:
            rospy.logerr("CvBridge Error: {0}".format(e))
    
        gray_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        edges_img = cv2.Canny(gray_img, self.canny_threshold1, self.canny_threshold2, apertureSize=self.canny_aperture)

        edges_ros_image = self.bridge.cv2_to_imgmsg(edges_img, encoding="mono8")
        self.cannyPub.publish(edges_ros_image)


        self.lines_publisher(cv_image, edges_img)
    
    def dynam_reconfigure_callback(self, config):
        self.canny_threshold1 = config.canny_threshold1
        self.canny_threshold2 = config.canny_threshold2
        self.canny_aperture = config.canny_aperture_size




if __name__ == '__main__':
    node = GateDetectionNode()

    while not rospy.is_shutdown():
        rospy.spin()

