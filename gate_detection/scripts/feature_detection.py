#!/usr/bin/env python

import numpy as np
import math
from collections import defaultdict
import cv2
import copy
from timeit import default_timer as timer

import icp

"""@package feature_detection
Object detection module using various feature detection algorithms.

Methods in this module use various pre-processing, processing, and filtering algorithms
to be able to detect more abstractly shaped objects e.g. gates, poles, sticks, etc.,
that may be difficult to detect using neural network based object detection methods. 

By BenG @ Vortex NTNU, 2022
"""


class FeatureDetection:
    """Class for every algorithm needed for feature-processing based object detection."""

    def hsv_processor(
        original_image,
        hsv_hue_min,
        hsv_hue_max,
        hsv_sat_min,
        hsv_sat_max,
        hsv_val_min,
        hsv_val_max,
    ):
        """Takes a raw image and applies Hue-Saturation-Value filtering.

        Params:
            original_image      (cv2::Mat)  : An image with BGR channels.
            hsv_hue_min         (uint8)     : Lower bound for hue filtering. Range: 0-179.
            hsv_hue_max         (uint8)     : Upper bound for hue filtering. Range: 0-179.
            hsv_sat_min         (uint8)     : Lower bound for saturation filtering. Range: 0-255.
            hsv_sat_max         (uint8)     : Upper bound for saturation filtering. Range: 0-255.
            hsv_val_min         (uint8)     : Lower bound for value filtering. Range: 0-255.
            hsv_val_max         (uint8)     : Upper bound for value filtering. Range: 0-255.

        Returns:
            hsv_img             (cv2::Mat)  : original_image converted into HSV color space.
            hsv_mask            (cv2::Mat)  : Array of x, y points for binary pixels that were filtered by the HSV params.
            hsv_mask_check_img  (cv2::Mat)  : original_image with applied hsv_mask.
        """
        orig_img_cp = copy.deepcopy(original_image)

        hsv_img = cv2.cvtColor(orig_img_cp, cv2.COLOR_BGR2HSV)
        hsv_lower = np.array([hsv_hue_min, hsv_sat_min, hsv_val_min])
        hsv_upper = np.array([hsv_hue_max, hsv_sat_max, hsv_val_max])

        hsv_mask = cv2.inRange(hsv_img, hsv_lower, hsv_upper)
        hsv_mask_check_img = cv2.bitwise_and(
            orig_img_cp, orig_img_cp, mask=hsv_mask
        )  # Applies mask

        return hsv_img, hsv_mask, hsv_mask_check_img

    def noise_removal_processor(
        hsv_mask,
        gb_kernel_size1,
        gb_kernel_size2,
        sigma,
        thresholding_blocksize,
        thresholding_C,
        erosion_dilation_kernel_size,
        erosion_iterations,
        dilation_iterations,
    ):
        """Applies various noise removal and morphism algorithms.
        Is meant to be used as a pre-processor for contour search.

        Params:
            hsv_mask                        (cv2::Mat)  : Array of x, y points for binary pixels that were filtered by the HSV params.
            gb_kernel_size1                 (uint8-odds): Vertical kernel size for Gaussian blur.
            gb_kernel_size2                 (uint8-odds): Horizontal kernel size for Gaussian blur.
            sigma                           (float32)   : Standard deviation to apply with Gaussian blur.
            thresholding_blocksize          (uint8-odds): Size of a pix neighborhood that is used to calculate a threshold value for the px.
            thresholding_C                  (uint8)     : Constant area subracted from the thresholded areas.
            erosion_dilation_kernel_size    (uint8-odds): Kernel size (resolution) for erosion and dilation algorithms.
            erosion_iterations              (uint8)     : The times to serially apply the erosion method.
            dilation_iterations             (uint8)     : The times to serially apply the dilation method.

        Returns:
            hsv_img             (cv2::Mat)  : original_image converted into HSV color space.
            hsv_mask            (cv2::Mat)  : Array of x, y points for binary pixels that were filtered by the HSV params.
            hsv_mask_check_img  (cv2::Mat)  : original_image with applied hsv_mask.
        """
        hsv_mask_cp = copy.deepcopy(hsv_mask)

        blur_hsv_img = cv2.GaussianBlur(
            hsv_mask_cp, (gb_kernel_size1, gb_kernel_size2), sigma
        )

        thr_img = cv2.adaptiveThreshold(
            blur_hsv_img,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV,
            thresholding_blocksize,
            thresholding_C,
        )

        erosion_dilation_kernel = np.ones(
            (erosion_dilation_kernel_size, erosion_dilation_kernel_size), np.uint8
        )
        erosion_img = cv2.erode(
            thr_img, erosion_dilation_kernel, iterations=erosion_iterations
        )
        dilation_img = cv2.dilate(
            erosion_img, erosion_dilation_kernel, iterations=dilation_iterations
        )

        return dilation_img

    def contour_filtering(hierarchy, contours, contour_area_threshold, mode=1):
        """Filters contours according to contour hierarchy and area.

        Params:
            hierarchy               (array[][]) : Hierarchy of the contours parameter. Must be of 'cv2.RETR_CCOMP' type. 
            contours                (array[][]) : Contours that are to be filtered.
            contour_area_threshold  (uint16)    : Threshold for filtering based on inside-of-contour area.
                                                  Contours with lower area than the argument will be removed.
            mode                    (uint8)     : {default=1} Mode for hierarchical filtering.
                                                  Mode 1 leaves only the contours that do not have any hierarchical children.
                                                  Mode 2 leaves the contours that do not have any hierarchical children or neighbours.

        Returns:
            hsv_img (array[][]): Filtered contours.
        """
        filtered_contours = []

        for cnt_idx in range(len(contours)):
            cnt_hier = hierarchy[0][cnt_idx]

            if mode == 1:
                if (
                    ((cnt_hier[0] == cnt_idx + 1) or (cnt_hier[0] == -1))
                    and ((cnt_hier[1] == cnt_idx - 1) or (cnt_hier[1] == -1))
                    and (cnt_hier[2] == -1)
                ):
                    cnt = contours[cnt_idx]
                    cnt_area = cv2.contourArea(cnt)
                    if cnt_area < contour_area_threshold:
                        filtered_contours.append(False)
                    else:
                        filtered_contours.append(True)
                else:
                    filtered_contours.append(False)

            if mode == 2:
                if len([i for i, j in zip(cnt_hier, [-1, -1, -1, cnt_idx - 1]) if i == j]) != 4:
                    cnt = contours[cnt_idx]
                    cnt_area = cv2.contourArea(cnt)
                    if cnt_area < contour_area_threshold:
                        filtered_contours.append(False)
                    else:
                        filtered_contours.append(True)
                else:
                    filtered_contours.append(False)

        return filtered_contours

    def contour_processing(original_image, noise_removed_img, enable_convex_hull=False):
        orig_img_cp = copy.deepcopy(original_image)
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