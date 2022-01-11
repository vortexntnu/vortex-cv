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

        self.zedSub = rospy.Subscriber('/zed2/zed_node/rgb/image_rect_color', Image, self.zed_callback)
        
        self.linesPub = rospy.Publisher('/gate_detection/lines_image', Image, queue_size= 1)
        self.cannyPub = rospy.Publisher('/gate_detection/canny_image', Image, queue_size= 1)
        self.hsvPub = rospy.Publisher('/gate_detection/hsv_image', Image, queue_size= 1)
        self.hsvCheckPub = rospy.Publisher('/gate_detection/hsv_check_image', Image, queue_size= 1)
        self.noiseRmPub = rospy.Publisher('/gate_detection/noise_removal_image', Image, queue_size= 1)
        self.contourPub = rospy.Publisher('/gate_detection/contour_image', Image, queue_size= 1)

        self.timerPub = rospy.Publisher('/gate_detection/timer', Float32, queue_size= 1)

        self.bridge = CvBridge()

        # Canny params
        self.canny_threshold1 = 100
        self.canny_threshold2 = 200
        self.canny_aperture = 3

        # HSV params
        self.hsv_hue_min = 179
        self.hsv_hue_max = 0
        self.hsv_sat_min = 255
        self.hsv_sat_max = 0
        self.hsv_val_min = 255
        self.hsv_val_max = 0

        # Blur params
        self.ksize1 = 7
        self.ksize2 = 7
        self.sigma = 0.8

        # Thresholding params
        self.thresholding_blocksize = 11
        self.thresholding_C = 2

        # Erosion and dilation params
        self.erosion_dilation_ksize = 5
        self.erosion_iterations = 1
        self.dilation_iterations = 1

        self.dynam_client = dynamic_reconfigure.client.Client("gate_detection_cfg", timeout=5.0, config_callback=self.dynam_reconfigure_callback)

    

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


    def hsv_publisher(self, orig_img):
        orig_img_cp = copy.deepcopy(orig_img)

        hsv_img = cv2.cvtColor(orig_img_cp, cv2.COLOR_BGR2HSV)
        hsv_lower = np.array([self.hsv_hue_min, self.hsv_sat_min, self.hsv_val_min])
        hsv_upper = np.array([self.hsv_hue_max, self.hsv_sat_max, self.hsv_val_max])
        
        # hsv_lower = np.array([0, 0, 0])
        # hsv_upper = np.array([10, 20, 20])
        
        hsv_mask = cv2.inRange(hsv_img, hsv_lower, hsv_upper)
        hsv_mask_check_img = cv2.bitwise_and(orig_img_cp, orig_img_cp, mask=hsv_mask)

        hsv_ros_image = self.bridge.cv2_to_imgmsg(hsv_img, encoding="bgr8")
        hsv_check_ros_image = self.bridge.cv2_to_imgmsg(hsv_mask_check_img, encoding="bgra8")
        
        self.hsvPub.publish(hsv_ros_image)
        self.hsvCheckPub.publish(hsv_check_ros_image)

        return hsv_mask
        # rospy.loginfo("\nHue: %d %d\nSat: %d %d\nVal: %d %d\n", self.hsv_hue_min, self.hsv_hue_max, self.hsv_sat_min, self.hsv_sat_max, self.hsv_val_min, self.hsv_val_max)

    def noise_removal(self, hsv_mask):
        hsv_mask_cp = copy.deepcopy(hsv_mask)

        blur_hsv_img = cv2.GaussianBlur(hsv_mask_cp, (self.ksize1, self.ksize2), self.sigma)
        
        thr_img = cv2.adaptiveThreshold(blur_hsv_img, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, self.thresholding_blocksize, self.thresholding_C)

        erosion_dilation_kernel = np.ones((self.erosion_dilation_ksize,self.erosion_dilation_ksize), np.uint8)
        erosion_img = cv2.erode(thr_img, erosion_dilation_kernel, iterations=self.erosion_iterations)
        dilation_img = cv2.dilate(erosion_img, erosion_dilation_kernel, iterations=self.dilation_iterations)

        noise_rm_ros_image = self.bridge.cv2_to_imgmsg(dilation_img, encoding="mono8")
        self.noiseRmPub.publish(noise_rm_ros_image)

        return dilation_img


    def contour_processing(self, orig_img, noise_removed_img):
        orig_img_cp = copy.deepcopy(orig_img)

        contours, hierarchy = cv2.findContours(noise_removed_img, 1, 2)
        print(contours)

        for cnt_idx in range(len(contours)):
            print("Contour: ")
            print(contours[cnt_idx])
            print("\n\n")
            cv2.drawContours(orig_img_cp, contours, cnt_idx, (0,0,255), 2)

            # rect = cv2.minAreaRect(cnt)
            # box = cv2.boxPoints(rect)
            # box = np.int0(box)
            # cv2.drawContours(orig_img_cp,[box],0,(0,0,255),2)

        # cnt = contours[0]
        # M = cv2.moments(cnt)
        # print( M )

        contour_image = self.bridge.cv2_to_imgmsg(orig_img_cp, encoding="bgra8")
        self.contourPub.publish(contour_image)

        return orig_img_cp

    def zed_callback(self, img_msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, "passthrough")
        except CvBridgeError, e:
            rospy.logerr("CvBridge Error: {0}".format(e))

        gray_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        canny_img = cv2.Canny(gray_img, self.canny_threshold1, self.canny_threshold2, apertureSize=self.canny_aperture)

        edges_ros_image = self.bridge.cv2_to_imgmsg(canny_img, encoding="mono8")
        self.cannyPub.publish(edges_ros_image)


        self.lines_publisher(cv_image, canny_img)
        hsv_mask = self.hsv_publisher(cv_image)

        noise_removed_img = self.noise_removal(hsv_mask)

        self.contour_processing(cv_image, noise_removed_img)


    def dynam_reconfigure_callback(self, config):
        self.canny_threshold1 = config.canny_threshold1
        self.canny_threshold2 = config.canny_threshold2
        self.canny_aperture = config.canny_aperture_size

        self.hsv_hue_min = config.hsv_hue_min
        self.hsv_hue_max = config.hsv_hue_max
        self.hsv_sat_min = config.hsv_sat_min
        self.hsv_sat_max = config.hsv_sat_max
        self.hsv_val_min = config.hsv_val_min
        self.hsv_val_max = config.hsv_val_max

        self.ksize1 = config.ksize1
        self.ksize2 = config.ksize2
        self.sigma = config.sigma

        self.thresholding_blocksize = config.blocksize
        self.thresholding_C = config.C

        self.erosion_dilation_ksize = config.ed_ksize
        self.erosion_iterations = config.erosion_iterations
        self.dilation_iterations = config.dilation_iterations

if __name__ == '__main__':
    node = GateDetectionNode()

    while not rospy.is_shutdown():
        rospy.spin()

