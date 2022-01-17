#!/usr/bin/env python

from asyncore import close_all
from cmath import sqrt
from enum import unique
from fileinput import close
from multiprocessing.sharedctypes import Value
from time import sleep
import rospy

from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Empty
from cv_msgs.msg import BBox

from cv_bridge import CvBridge, CvBridgeError
import dynamic_reconfigure.client

import numpy as np
import math
from collections import defaultdict
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
        self.resetSub = rospy.Subscriber('/gate_detection/reset', Empty, self.gate_detection_reset_callback)
        
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
        self.filteredRectPub = rospy.Publisher('/gate_detection/filtered_rect_image', Image, queue_size= 1)
        self.filteredRectMaskPub = rospy.Publisher('/gate_detection/filtered_rect_mask_image', Image, queue_size= 1)
        self.BBoxPub = rospy.Publisher('/gate_detection/bbox_image', Image, queue_size= 1)
        
        self.classificationPub = rospy.Publisher('/gate_detection/bbox', Image, queue_size= 1)

        self.timerPub = rospy.Publisher('/gate_detection/timer', Float32, queue_size= 1)

        self.bridge = CvBridge()

        # point filtering
        self.prev_closest_points = []
        self.prev_closest_point_dsts = []
        
        self.integral_diff_values_arr = []
        self.integral_diff_values_arr_len = 5

        self.ref_points_icp_fitting_base = np.array([[449, 341], [845, 496], [690, 331]], dtype=int)
        self.ref_points_icp_fitting = np.array([[449, 341], [845, 496], [690, 331]], dtype=int)

        self.classified_gate = False

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


    def cv_image_publisher(self, publisher, image, msg_encoding="bgra8"):
        """
        Takes a cv::Mat image object, converts it into a ROS Image message type, and publishes it using the specified publisher.
        """
        msgified_img = self.bridge.cv2_to_imgmsg(image, encoding=msg_encoding)
        publisher.publish(msgified_img)


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
            cv2.drawContours(orig_img_cp, using_contours, cnt_idx, (255,0,0), 2)

            centroid_data.append((centroid_center_x, centroid_center_y, cnt_area))
            cnt_area_str = str(centroid_data[cnt_idx][2])

            orig_img_cp = cv2.circle(orig_img_cp, (centroid_data[cnt_idx][0], centroid_data[cnt_idx][1]), 2, (0,255,0), 2)
            # orig_img_cp = cv2.putText(orig_img_cp, cnt_area_str, (centroid_data[cnt_idx][0], centroid_data[cnt_idx][1]), cv2.FONT_HERSHEY_SIMPLEX, 1, (1,0,0), 2)

        contour_image = self.bridge.cv2_to_imgmsg(orig_img_cp, encoding="bgra8")
        if not enable_convex_hull:
            self.contourPub.publish(contour_image)
        else:
            self.hullPub.publish(contour_image)
        return orig_img_cp, using_contours


    def shape_fitting(self, orig_img, contours, fit_threshold):
        orig_img_cp = copy.deepcopy(orig_img)

        blank_image = np.zeros(shape=[self.img_height, self.img_width, self.img_channels], dtype=np.uint8)
        fitted_boxes = []

        for cnt in contours:
            if len(cnt) > 20:
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
                    orig_img_cp = cv2.drawContours(orig_img_cp,[box],0,(0,0,255),2)
                    fitted_boxes.append(box)
            
                # rect_area = rect[1][0] * rect[1][1]
                # cnt_area = cv2.contourArea(cnt)
                # diff_area = rect_area - cnt_area
                
                # if diff_area < (cnt_area * fit_threshold):
                #     box = cv2.boxPoints(rect)
                #     box = np.int0(box)
                #     orig_img_cp = cv2.drawContours(orig_img_cp,[box],0,(0,0,255),2)
        
        shapes_ros_image = self.bridge.cv2_to_imgmsg(orig_img_cp, encoding="bgra8")
        self.shapePub.publish(shapes_ros_image)

        return fitted_boxes, orig_img_cp

    
    def icp_fitting(self, orig_img, fitted_boxes, ref_points):
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
        
        # print(centroid_arr)
        # print(A2D)
        
        T_h, icp_points = icp.icp(centroid_arr, ref_points, verbose=False)
        points_int = np.rint(icp_points)
        # print(points_int)
        # print("\n")
        for pnt in icp_points:
            orig_img_cp = cv2.circle(orig_img_cp, (int(pnt[0]), int(pnt[1])), 2, (0,255,0), 2)
        
        fitted_points_ros_image = self.bridge.cv2_to_imgmsg(orig_img_cp, encoding="bgra8")
        self.fittedPointsPub.publish(fitted_points_ros_image)

        return icp_points, centroid_arr
    
    def point_distances(self, point_arr1, point_arr2):
        # Find distance from every reference point (array 1) to every point in array 2
        # Stores values in table formatted: array A of len(array_1), 
        # where every elem in A is array B of len(array_2) of distances from point idxed in A according to array 1 to point idxed in B according to array 2
        number_reference_points = len(point_arr1)
        distance_table = []

        for p_n_idx in range(len(point_arr1)):
            p_n = point_arr1[p_n_idx]
            distance_table.append([])
            for p_k_idx in range(len(point_arr2)):
                p_k = point_arr2[p_k_idx]

                dst2point = math.sqrt((abs(p_n[0] - p_k[0]) ** 2) + (abs(p_n[1] - p_k[1]) ** 2)) 
                distance_table[p_n_idx].append(dst2point)
        return distance_table

    def custom_closest_point(self, point_arr1, point_arr2):
        # Gives the closest point and the distance to the point from point a in array 1 to point b in array 2
        distance_table = self.point_distances(point_arr1, point_arr2)

        closest_point_idxes = []
        closest_points = []
        closest_point_dsts = []

        # for each reference point the closest point is written to the idx of the reference points list
        for dsts_2_ref_point in distance_table:
            closest_point_idx = min(range(len(dsts_2_ref_point)), key=dsts_2_ref_point.__getitem__)
            closest_point_idxes.append(closest_point_idx)            
            closest_point_dsts.append(round(dsts_2_ref_point[closest_point_idx], 4))

        for point_idx in closest_point_idxes:
            closest_points.append(point_arr2[point_idx])

        return closest_points, closest_point_dsts
    
    def get_duplicate_points(self, a):
        seen = set()
        dupes = []

        for x in a:
            if x in seen:
                dupes.append(x)
            else:
                seen.add(x)
        
        return dupes
    
    def duplicate_point_filter(self, closest_points, closest_point_dsts):
        # closest_points_np = np.rint(np.array([[2, 2], [3, 3], [4, 4], [3, 3], [2, 2], [1, 1]]))
        closest_points_np = np.rint(np.array(closest_points))

        # An index in indices is the same index in closest_points, and a value in indices is an index for a value in uniq_points, that is a value in closest_points with same index as indices 
        uniq_points, indices = np.unique(closest_points_np, return_inverse=True, axis=0)

        num_closest_points = len(closest_points_np)
        num_uniq_closest_points = len(uniq_points)
        
        if num_closest_points == num_uniq_closest_points:
            return closest_points, closest_point_dsts

        # Duplicate point filter starts here
        indices_of_indices = defaultdict(list)
        for i,item in enumerate(indices):
            indices_of_indices[item].append(i)
        indices_of_indices = {index_of_val_in_uniq_points:index_of_point_in_closest_points for index_of_val_in_uniq_points,index_of_point_in_closest_points \
                              in indices_of_indices.items() if len(index_of_point_in_closest_points)>1}

        for point_val_in_uniq_points, indices_of_p1 in indices_of_indices.items():
            comp_dsts = []
            for not_closest_point_idx in indices_of_p1:
                comp_dsts.append(closest_point_dsts[not_closest_point_idx])
            actual_closest_point_idx_in_indices_of_p1 = min(range(len(comp_dsts)), key=comp_dsts.__getitem__)
            actual_closest_point_idx = indices_of_p1[actual_closest_point_idx_in_indices_of_p1]
            
            indices_of_p1.pop(actual_closest_point_idx_in_indices_of_p1)

            for not_closest_point_idx in indices_of_p1:
                closest_points[not_closest_point_idx] = self.prev_closest_points[not_closest_point_idx]
                closest_point_dsts[not_closest_point_idx] = self.prev_closest_point_dsts[not_closest_point_idx]

        return closest_points, closest_point_dsts
    
    def point_thresholding(self, closest_points, closest_point_dsts, threshold, reset_reference_points_threshold):
        pts_cp = copy.deepcopy(closest_points)
        pt_dsts_cp = copy.deepcopy(closest_point_dsts)
        diff_dsts = []
        for i in range(len(self.prev_closest_point_dsts)):
            closest_pt_dst = pt_dsts_cp[i]
            prev_closest_pt_dst = self.prev_closest_point_dsts[i]

            diff_prev_current_dst = abs(prev_closest_pt_dst - closest_pt_dst)
            diff_dsts.append(diff_prev_current_dst)
            
            if diff_prev_current_dst > threshold:
                pts_cp[i] = self.prev_closest_points[i]
                pt_dsts_cp[i] = self.prev_closest_point_dsts[i]
                # rospy.loginfo("Point changed position too rapidly! Change: %f", diff_prev_current_dst)

            integral_check = (sum(self.integral_diff_values_arr) // self.integral_diff_values_arr_len)
            # print(integral_check, len(self.integral_diff_values_arr))
            if integral_check > reset_reference_points_threshold:
                rospy.loginfo("Reset! I-arr len: %d    I-arr value: %d", len(self.integral_diff_values_arr), integral_check)
                self.gate_detection_reset()
        try:
            self.integral_diff_values_arr.append(max(diff_dsts))
            if len(self.integral_diff_values_arr) > self.integral_diff_values_arr_len:
                self.integral_diff_values_arr.pop(0)
        except ValueError:
            pass

        return pts_cp, pt_dsts_cp, diff_dsts

    def reference_points_iteration(self, closest_points):
        self.ref_points_icp_fitting = np.array(closest_points, dtype=int)
        # print(self.ref_points_icp_fitting)

    
    def fitted_point_filtering(self, point_arr1, point_arr2):
        closest_points, closest_point_dsts = self.custom_closest_point(point_arr1, point_arr2)
        
        closest_points_filtered, closest_point_dsts_filtered = self.duplicate_point_filter(closest_points, closest_point_dsts)

        thresholded_closest_points, thresholded_closest_point_dsts, diff_dsts = self.point_thresholding(closest_points_filtered,
                                                                                                        closest_point_dsts_filtered,
                                                                                                        threshold=50,
                                                                                                        reset_reference_points_threshold=100)
        # Sometimes makes it better, sometimes not
        self.reference_points_iteration(thresholded_closest_points)

        self.prev_closest_points = thresholded_closest_points
        self.prev_closest_point_dsts = thresholded_closest_point_dsts


        
        return thresholded_closest_points


    def get_contour_from_rect(self, rect):
        return np.array(rect).reshape((-1,1,2)).astype(np.int32)


    def does_ctr_contain_point(self, ctr, point):
        indicator = cv2.pointPolygonTest(ctr, tuple(point), measureDist=False)
        if indicator >= 0:
            return True
        else:
            return False


    def get_relevant_rects(self, point_arr, rect_arr):
        relevant_rects = []
        for rect in rect_arr:
            ctr = self.get_contour_from_rect(rect)
            for point in point_arr:
                is_in_rect = self.does_ctr_contain_point(ctr, point)
                if is_in_rect:
                    relevant_rects.append(rect)
        
        return relevant_rects


    def get_all_points_in_rects(self, rects):
        # Not optimized at all lol
        # Possible for rectangle-wise point extraction in this fnc (mv np.zeros blank img to rect in rects loop)
        blank_image = np.zeros(shape=[self.img_height, self.img_width, self.img_channels], dtype=np.uint8)
        
        rects_arr_shape = np.shape(rects)
        num_of_rects = rects_arr_shape[0]
        
        # points_in_rects = []
        # for i in range(num_of_rects):
        #     points_in_rects.append([])

        for rect in rects:
            ctr = self.get_contour_from_rect(rect)
            cv2.drawContours(blank_image, [ctr], 0, (255,255,255), thickness=cv2.FILLED)
        
        blank_image = cv2.cvtColor(blank_image, cv2.COLOR_BGR2GRAY)

        px_arr = np.argwhere(blank_image == 255)
        # print(px_arr)
        
        filtered_rect_mask_ros_image = self.bridge.cv2_to_imgmsg(blank_image, encoding="mono8")
        self.filteredRectMaskPub.publish(filtered_rect_mask_ros_image)
        
        return px_arr, blank_image


    def rect_filtering(self, img, fitted_box_centers, icp_fitted_points, fitted_boxes):
        img_cp = copy.deepcopy(img)
        blank_image = np.zeros(shape=[self.img_height, self.img_width, self.img_channels], dtype=np.uint8)

        closest_points, closest_point_dsts = self.custom_closest_point(icp_fitted_points, fitted_box_centers)
        closest_points = self.fitted_point_filtering(icp_fitted_points, fitted_box_centers)

        relevant_rects = self.get_relevant_rects(closest_points, fitted_boxes)
        points_in_rects, imgimg = self.get_all_points_in_rects(relevant_rects)

        # for pnt in fitted_box_centers:
        #     cv2.circle(img_cp, (int(pnt[0]), int(pnt[1])), 3, (0,0,255), 10)
        
        for pnt in closest_points:
            cv2.circle(img_cp, (int(pnt[0]), int(pnt[1])), 5, (255,0,255), 2)
        
        for box in relevant_rects:
            box = np.int0(box)
            cv2.drawContours(img_cp,[box],0,(0,0,255),2)
        
        """ for cx, cy, h, w, phi in fitted_boxes:
            if blank_image[cx - w//2: cx + w//2][cy - h//2: cy + h//2]: """

        filtered_rect_ros_image = self.bridge.cv2_to_imgmsg(img_cp, encoding="bgra8")
        self.filteredRectPub.publish(filtered_rect_ros_image)

        return relevant_rects, closest_points, fitted_box_centers, points_in_rects
    

    def create_bbox(self, img, points_in_rects):
        img_cp = copy.deepcopy(img)
        blank_image = np.zeros(shape=[self.img_height, self.img_width, self.img_channels], dtype=np.uint8)

        x_lst, y_lst = zip(*points_in_rects)
        
        xmin = min(x_lst)
        ymin = min(y_lst)
        xmax = max(x_lst)
        ymax = max(y_lst)
        
        cv2.rectangle(img_cp,(ymin, xmin),(ymax, xmax),(0,255,0),2)
        cv2.putText(img_cp, 'GATE', (ymin, xmin-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (36,255,12), 2)

        rect_area = (ymax - ymin) * (xmax - xmin)

        if self.classified_gate:
            bbox_ros_image = self.bridge.cv2_to_imgmsg(img_cp, encoding="bgra8")
        else:
            bbox_ros_image = self.bridge.cv2_to_imgmsg(img, encoding="bgra8")
        self.BBoxPub.publish(bbox_ros_image)


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


    def line_fitting(self, orig_img, contours, threshold=50):
        orig_img_cp = copy.deepcopy(orig_img)

        blank_image = np.zeros(shape=[self.img_height, self.img_width, self.img_channels], dtype=np.uint8)
        # cv2.fillPoly(blank_image, pts =contours, color=(255,255,255))
        theta_set = set()
        for cnt in contours:
            rows, cols = blank_image.shape[:2]
            [vx,vy,x,y] = cv2.fitLine(cnt, cv2.DIST_L2,0,0.01,0.01)
            theta = math.atan(vy/vx)
            theta = (abs(theta*math.pi*180) % 360)
            theta_set.add(theta)
            lefty = int((-x*vy/vx) + y)
            righty = int(((cols-x)*vy/vx)+y)
            cv2.line(blank_image,(cols-1,righty),(0,lefty),(0,255,0),4)
            cv2.putText(blank_image, str(theta), (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)

            #cv2.line(orig_img_cp,(cols-1,righty),(0,lefty),(0,255,0),2)
        
        theta_arr = list(theta_set)
        parallell_line_count = 0
        for theta_outer in theta_arr:
            if 90 < theta_outer < 180:
                for theta_inner in theta_arr:
                    if (theta_outer + threshold >= theta_inner) and (theta_outer - threshold <= theta_inner) and (theta_outer != theta_inner):
                        parallell_line_count += 1
            else:
                continue
        
        lines_ros_image = self.bridge.cv2_to_imgmsg(blank_image, encoding="bgra8")
        self.linesPub.publish(lines_ros_image)

        return blank_image, parallell_line_count
    
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

        fitted_boxes, shape_img = self.shape_fitting(cv_image, contours, 5)
        
        icp_points, centroid_arr = self.icp_fitting(cv_image, fitted_boxes, self.ref_points_icp_fitting)
        relevant_rects, closest_points, fitted_box_centers, points_in_rects = self.rect_filtering(cv_image, centroid_arr, icp_points, fitted_boxes)
        line_img, parallell_line_count = self.line_fitting(contours_img, relevant_rects)
        if parallell_line_count > 1:
            self.classified_gate = True

        bbox_img = self.create_bbox(cv_image, points_in_rects)

        self.classified_gate = False
        # self.convex_fitting(contours_img, contours, hull_contours_img, hull_contours, 0.4)
        # self.corner_detection(line_img)
        #------------------------------------------>
        #------------------------------------------>
        #------------------------------------------>
        end = timer() # Stop function timer.
        timediff = (end - start)

        fps = 1 / timediff # Take reciprocal of the timediff to get runs per second. 

        self.timerPub.publish(fps)


    def gate_detection_reset(self):
        self.ref_points_icp_fitting = self.ref_points_icp_fitting_base
        self.prev_closest_points = []
        self.prev_closest_point_dsts = []        
        self.integral_diff_values_arr = []


    def gate_detection_reset_callback(self, msg):
        self.gate_detection_reset()


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

