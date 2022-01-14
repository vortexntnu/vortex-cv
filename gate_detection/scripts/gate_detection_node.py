#!/usr/bin/env python

from time import sleep
import rospy

from sensor_msgs.msg import Image
from std_msgs.msg import Float32

from cv_bridge import CvBridge, CvBridgeError
import dynamic_reconfigure.client

import numpy as np
import cv2
import copy
from timeit import default_timer as timer

import icp

np.set_printoptions(threshold=np.inf)

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
        self.hullPub = rospy.Publisher('/gate_detection/hull_image', Image, queue_size= 1)
        self.cornersPub = rospy.Publisher('/gate_detection/corners_image', Image, queue_size= 1)
        self.shapePub = rospy.Publisher('/gate_detection/shapes_image', Image, queue_size= 1)
        self.cvxFitPub = rospy.Publisher('/gate_detection/convex_fitting_image', Image, queue_size= 1)
        self.fittedPointsPub = rospy.Publisher('/gate_detection/fitted_points_image', Image, queue_size= 1)

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

        blur_hsv_img = cv2.GaussianBlur(hsv_mask_cp, (9, 17), 10)
        # blur_hsv_img = cv2.GaussianBlur(hsv_mask_cp, (self.ksize1, self.ksize2), self.sigma)
        
        thr_img = cv2.adaptiveThreshold(blur_hsv_img, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, self.thresholding_blocksize, self.thresholding_C)

        erosion_dilation_kernel = np.ones((self.erosion_dilation_ksize,self.erosion_dilation_ksize), np.uint8)
        erosion_img = cv2.erode(thr_img, erosion_dilation_kernel, iterations=self.erosion_iterations)
        dilation_img = cv2.dilate(erosion_img, erosion_dilation_kernel, iterations=self.dilation_iterations)

        noise_rm_ros_image = self.bridge.cv2_to_imgmsg(dilation_img, encoding="mono8")
        self.noiseRmPub.publish(noise_rm_ros_image)

        return dilation_img


    def contour_filtering(self, hierarchy, contours):
        cnt_filter = []

        for cnt_idx in range(len(contours)):
            cnt_hier = hierarchy[0][cnt_idx]

            # if len([i for i, j in zip(hierarchy[0][cnt_idx], [-1, -1, -1, cnt_idx - 1]) if i == j]) != 4:

            if ((cnt_hier[0] == cnt_idx + 1) or (cnt_hier[0] == -1)) and ((cnt_hier[1] == cnt_idx - 1) or (cnt_hier[1] == -1)) and (cnt_hier[2] == -1):
                cnt = contours[cnt_idx]
                cnt_area = cv2.contourArea(cnt)
                if cnt_area < 300:
                    cnt_filter.append(False)
                else:
                    cnt_filter.append(True)
            else:
                cnt_filter.append(False)
        return cnt_filter


    def contour_processing(self, orig_img, noise_removed_img, enable_convex_hull=False):
        orig_img_cp = copy.deepcopy(orig_img)
        blank_image = np.zeros(shape=[self.img_height, self.img_width, self.img_channels], dtype=np.uint8)

        # contours, hierarchy = cv2.findContours(noise_removed_img, 1, 2)
        # contours, hierarchy = cv2.findContours(noise_removed_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours, hierarchy = cv2.findContours(noise_removed_img, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)

        cnt_fiter = self.contour_filtering(hierarchy, contours)
        contours_array = np.array(contours)
        contours_filtered = contours_array[cnt_fiter]

        using_contours = []
        if enable_convex_hull:
            hull_array = []
            for cnt_idx in range(len(contours_filtered)):
                hull_array.append(cv2.convexHull(contours_filtered[cnt_idx], False))
            using_contours = hull_array
        else:
            using_contours = contours_filtered


        centroid_data = []
        for cnt_idx in range(len(contours_filtered)):

            cnt = using_contours[cnt_idx]
            cnt_moments = cv2.moments(cnt)

            centroid_center_x = int(cnt_moments['m10']/cnt_moments['m00'])
            centroid_center_y = int(cnt_moments['m01']/cnt_moments['m00'])

            cnt_area = cnt_moments['m00']
            cv2.drawContours(blank_image, using_contours, cnt_idx, (255,0,0), 2)

            centroid_data.append((centroid_center_x, centroid_center_y, cnt_area))
            cnt_area_str = str(centroid_data[cnt_idx][2])

            orig_img_cp = cv2.circle(orig_img_cp, (centroid_data[cnt_idx][0], centroid_data[cnt_idx][1]), 2, (0,255,0), 2)
            orig_img_cp = cv2.putText(orig_img_cp, cnt_area_str, (centroid_data[cnt_idx][0], centroid_data[cnt_idx][1]), cv2.FONT_HERSHEY_SIMPLEX, 1, (1,0,0), 2)

        contour_image = self.bridge.cv2_to_imgmsg(blank_image, encoding="bgra8")
        if not enable_convex_hull:
            self.contourPub.publish(contour_image)
        else:
            self.hullPub.publish(contour_image)
        return blank_image, using_contours


    def shape_fitting(self, orig_img, contours, fit_threshold):
        orig_img_cp = copy.deepcopy(orig_img)

        blank_image = np.zeros(shape=[self.img_height, self.img_width, self.img_channels], dtype=np.uint8)
        fitted_boxes = []

        for cnt in contours:
            rect = cv2.minAreaRect(cnt)


            rect_width = rect[1][0]
            rect_height = rect[1][1]
            
            rect_long = rect_height
            rect_short = rect_width

            if rect_height < rect_width:
                rect_long = rect_width
                rect_short = rect_height
            
            if rect_long > rect_short * fit_threshold:
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                blank_image = cv2.drawContours(blank_image,[box],0,(0,0,255),2)
                fitted_boxes.append(box)
        
            # rect_area = rect[1][0] * rect[1][1]
            # cnt_area = cv2.contourArea(cnt)
            # diff_area = rect_area - cnt_area
            
            # if diff_area < (cnt_area * fit_threshold):
            #     box = cv2.boxPoints(rect)
            #     box = np.int0(box)
            #     orig_img_cp = cv2.drawContours(orig_img_cp,[box],0,(0,0,255),2)
        
        shapes_ros_image = self.bridge.cv2_to_imgmsg(blank_image, encoding="bgra8")
        self.shapePub.publish(shapes_ros_image)

        return fitted_boxes

    
    def icp_fitting(self, orig_img, fitted_boxes):
        orig_img_cp = copy.deepcopy(orig_img)

        blank_image = np.zeros(shape=[self.img_height, self.img_width, self.img_channels], dtype=np.uint8)
        centroid_arr = np.empty([len(fitted_boxes), 2], dtype=int)

        for cnt_idx in range(len(fitted_boxes)):
            cnt = fitted_boxes[cnt_idx]
            cnt_moments = cv2.moments(cnt)

            centroid_center_x = int(cnt_moments['m10']/cnt_moments['m00'])
            centroid_center_y = int(cnt_moments['m01']/cnt_moments['m00'])

            cnt_area = cnt_moments['m00']

            centroid_arr[cnt_idx] = [centroid_center_x, centroid_center_y]

            # cv2.drawContours(blank_image, fitted_boxes, cnt_idx, (255,0,0), 2)
            # blank_image = cv2.circle(blank_image, (centroid_center_x, centroid_center_y), 2, (0,0,255), 2)
        
        A2D = np.array([[449, 341], [845, 496], [690, 331]], dtype=int)
        # print(centroid_arr)
        # print(A2D)
        
        T_h, points = icp.icp(centroid_arr, A2D, verbose=True)
        points_int = np.rint(points)
        # print(points_int)
        # print("\n")
        for pnt in points:
            orig_img_cp = cv2.circle(orig_img_cp, (int(pnt[0]), int(pnt[1])), 2, (0,255,0), 2)
        
        fitted_points_ros_image = self.bridge.cv2_to_imgmsg(orig_img_cp, encoding="bgra8")
        self.fittedPointsPub.publish(fitted_points_ros_image)
        

    def convex_fitting(self, contours_image, contours, convex_image, convex_contours, fit_threshold):
        contours_image = copy.deepcopy(contours_image)

        blank_image = np.zeros(shape=[self.img_height, self.img_width, self.img_channels], dtype=np.uint8)

        for cnt_idx in range(len(contours)):
            cnt = contours[cnt_idx]
            cvx = convex_contours[cnt_idx]

            cvx_area = cv2.contourArea(cvx)
            cnt_area = cv2.contourArea(cnt)
            diff_area = cvx_area - cnt_area
            
            if diff_area < (cnt_area * fit_threshold):
                cv2.drawContours(contours_image, convex_contours, cnt_idx, (0,255,0), 2)

        cvx_ros_image = self.bridge.cv2_to_imgmsg(contours_image, encoding="bgra8")
        self.cvxFitPub.publish(cvx_ros_image)


    def line_fitting(self, orig_img, contours):
        orig_img_cp = copy.deepcopy(orig_img)

        blank_image = np.zeros(shape=[self.img_height, self.img_width, self.img_channels], dtype=np.uint8)
        # cv2.fillPoly(blank_image, pts =contours, color=(255,255,255))
        
        for cnt in contours:
            rows, cols = blank_image.shape[:2]
            [vx,vy,x,y] = cv2.fitLine(cnt, cv2.DIST_L2,0,0.01,0.01)
            lefty = int((-x*vy/vx) + y)
            righty = int(((cols-x)*vy/vx)+y)
            cv2.line(blank_image,(cols-1,righty),(0,lefty),(0,255,0),2)
            #cv2.line(orig_img_cp,(cols-1,righty),(0,lefty),(0,255,0),2)
        
        lines_ros_image = self.bridge.cv2_to_imgmsg(blank_image, encoding="bgra8")
        self.linesPub.publish(lines_ros_image)

        return blank_image
    
    def corner_detection(self, line_fitted_img):
        line_fitted_img_cp = copy.deepcopy(line_fitted_img)
        #blur_line_fitted_img = cv2.GaussianBlur(line_fitted_img_cp, (5, 19), 5.2)

        blank_image = np.zeros(shape=[self.img_height, self.img_width, self.img_channels], dtype=np.uint8)
        blank_image_corners = np.zeros(shape=[self.img_height, self.img_width, self.img_channels], dtype=np.uint8)


        gray = cv2.cvtColor(line_fitted_img_cp,cv2.COLOR_BGR2GRAY)
        gray = np.float32(gray)
        dst = cv2.cornerHarris(gray,13,19,0.04)

        #result is dilated for marking the corners, not important
        dst = cv2.dilate(dst,None)

        # Threshold for an optimal value, it may vary depending on the image.
        blank_image[dst>0.00001*dst.max()]=[0,0,255,0]
        blank_image = cv2.cvtColor(blank_image,cv2.COLOR_BGR2GRAY)

        contours, hierarchy = cv2.findContours(blank_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            cnt_moments = cv2.moments(cnt)

            centroid_center_x = int(cnt_moments['m10']/cnt_moments['m00'])
            centroid_center_y = int(cnt_moments['m01']/cnt_moments['m00'])

            cv2.circle(blank_image_corners, (centroid_center_x, centroid_center_y), 2, (255,0,255), 2)

        corners_ros_img = self.bridge.cv2_to_imgmsg(blank_image_corners, encoding="bgra8")
        self.cornersPub.publish(corners_ros_img)


    def zed_callback(self, img_msg):
        start = timer() # Start function timer.
        #------------------------------------------>
        #------------------------------------------>
        #------------------------------------------>
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, "passthrough")
        except CvBridgeError, e:
            rospy.logerr("CvBridge Error: {0}".format(e))
        self.img_height, self.img_width, self.img_channels = cv_image.shape

        # gray_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # canny_img = cv2.Canny(gray_img, self.canny_threshold1, self.canny_threshold2, apertureSize=self.canny_aperture)
        # edges_ros_image = self.bridge.cv2_to_imgmsg(canny_img, encoding="mono8")
        # self.cannyPub.publish(edges_ros_image)
        # self.lines_publisher(cv_image, canny_img)
        
        hsv_mask = self.hsv_publisher(cv_image)
        noise_removed_img = self.noise_removal(hsv_mask)
        contours_img, contours = self.contour_processing(cv_image, noise_removed_img, False)
        # hull_contours_img, hull_contours = self.contour_processing(cv_image, noise_removed_img, True)

        fitted_boxes = self.shape_fitting(contours_img, contours, 4)
        
        self.icp_fitting(cv_image, fitted_boxes)

        # self.convex_fitting(contours_img, contours, hull_contours_img, hull_contours, 0.4)
        # line_img = self.line_fitting(contours_img, fitted_boxes)

        # self.corner_detection(line_img)
        #------------------------------------------>
        #------------------------------------------>
        #------------------------------------------>
        end = timer() # Stop function timer.
        timediff = (end - start)

        fps = 1 / timediff # Take reciprocal of the timediff to get runs per second. 

        self.timerPub.publish(fps)

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

