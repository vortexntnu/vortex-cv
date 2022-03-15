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


class ImageFeatureProcessing(object):
    def __init__(self, image_shape, *args, **kwargs):
        self.image_shape = image_shape

        super(ImageFeatureProcessing, self).__init__(*args, **kwargs)

    def hsv_processor(
        self,
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
            original_image      (cv2::Mat)  : An image with BGRA channels.
            hsv_hue_min         (uint8)     : Lower bound for hue filtering. Range: 0-179.
            hsv_hue_max         (uint8)     : Upper bound for hue filtering. Range: 0-179.
            hsv_sat_min         (uint8)     : Lower bound for saturation filtering. Range: 0-255.
            hsv_sat_max         (uint8)     : Upper bound for saturation filtering. Range: 0-255.
            hsv_val_min         (uint8)     : Lower bound for value filtering. Range: 0-255.
            hsv_val_max         (uint8)     : Upper bound for value filtering. Range: 0-255.

        Returns:
            hsv_img                 (cv2::Mat)  : original_image converted into HSV color space.
            hsv_mask                (cv2::Mat)  : Array of x, y points for binary pixels that were filtered by the HSV params.
            hsv_mask_validation_img (cv2::Mat)  : original_image with applied hsv_mask.
        """
        orig_img_cp = copy.deepcopy(original_image)
    
        hsv_img = cv2.cvtColor(original_image, cv2.COLOR_BGR2HSV)
        hsv_lower = np.array([hsv_hue_min, hsv_sat_min, hsv_val_min])
        hsv_upper = np.array([hsv_hue_max, hsv_sat_max, hsv_val_max])

        hsv_mask = cv2.inRange(hsv_img, hsv_lower, hsv_upper)
        hsv_mask_validation_img = cv2.bitwise_and(
            orig_img_cp, orig_img_cp, mask=hsv_mask
        )  # Applies mask

        return hsv_img, hsv_mask, hsv_mask_validation_img

    def noise_removal_processor(
        self,
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
            hsv_mask                        (cv2::Mat)  : A mono8 (8UC1) image that has the filtered HSV pixels.
            gb_kernel_size1                 (uint8-odds): Vertical kernel size for Gaussian blur.
            gb_kernel_size2                 (uint8-odds): Horizontal kernel size for Gaussian blur.
            sigma                           (float32)   : Standard deviation to apply with Gaussian blur.
            thresholding_blocksize          (uint8-odds): Size of a pix neighborhood that is used to calculate a threshold value for the px.
            thresholding_C                  (uint8)     : Constant area subracted from the thresholded areas.
            erosion_dilation_kernel_size    (uint8-odds): Kernel size (resolution) for erosion and dilation algorithms.
            erosion_iterations              (uint8)     : The times to serially apply the erosion method.
            dilation_iterations             (uint8)     : The times to serially apply the dilation method.

        Returns:
            morphised_image (cv2::Mat): Passed HSV image with morphised features using blur, thresholding, erosion, dilation.
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
        noise_removed_img = cv2.dilate(
            erosion_img, erosion_dilation_kernel, iterations=dilation_iterations
        )

        return noise_removed_img

    def _contour_filtering(
        self,
        contours,
        hierarchy,
        contour_area_threshold,
        contour_len_threshold=20,
        mode=1,
    ):
        """Filters contours according to contour hierarchy and area.

        Params:
            contours                (array[][]) : Contours that are to be filtered.
            hierarchy               (array[][]) : Hierarchy of the contours parameter. Must be of 'cv2.RETR_CCOMP' type.
            contour_area_threshold  (uint16)    : Threshold for filtering based on inside-of-contour area.
                                                  Contours with lower area than the argument will be removed.
            contour_len_threshold   (uint16)    : Threshold for filtering based on length of a contour.
            mode                    (uint8)     : {default=1} Mode for hierarchical filtering.
                                                  Mode 1 leaves only the contours that do not have any hierarchical children.
                                                  Mode 2 leaves the contours that do not have any hierarchical children or neighbours.

        Returns:
            filtered_contours (array[][]): Filtered contours.
        """
        filtered_contours = []
        try:
            num_of_contours = len(contours)
        except TypeError:
            return

        try:
            for cnt_idx in range(num_of_contours):
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
                            if len(cnt) > contour_len_threshold:
                                filtered_contours.append(True)
                            else:
                                filtered_contours.append(False)
                    else:
                        filtered_contours.append(False)

                if mode == 2:
                    if (
                        len(
                            [
                                i
                                for i, j in zip(cnt_hier, [-1, -1, -1, cnt_idx - 1])
                                if i == j
                            ]
                        )
                        != 4
                    ):
                        cnt = contours[cnt_idx]
                        cnt_area = cv2.contourArea(cnt)
                        if cnt_area < contour_area_threshold:
                            filtered_contours.append(False)
                        else:
                            if len(cnt) > contour_len_threshold:
                                filtered_contours.append(True)
                            else:
                                filtered_contours.append(False)
                    else:
                        filtered_contours.append(False)
        except ValueError:
            return

        return filtered_contours

    def contour_processing(
        self,
        noise_removed_image,
        contour_area_threshold,
        enable_convex_hull=False,
        return_image=True,
        image=None,
        show_centers=True,
        show_areas=False,
    ):
        """Finds contours in a pre-processed image and filters them.

        Params:
            noise_removed_image     (cv::Mat)   : A mono8 (8UC1) pre-processed image with morphised edges.
            contour_area_threshold  (uint16)    : Threshold for filtering based on inside-of-contour area.
                                                  Contours with lower area than the argument will be removed.
            enable_convex_hull      (bool)      : Enable convex hull contour approximation method.
            return_image            (bool)      : {default=True}False to return only contour data.
                                                  If param 'image' is none - returns a blanked image with drawn contour data.
                                                  If param 'image' is an image - returns both drawn blanked and passed images.
            image                   (cv::Mat)   : An image on which to draw processed contours.
            show_centers            (bool)      : Draw contour centers in the returned image(s).
            show_areas              (bool)      : Draw contour areas in the returned image(s).

        Returns:
                                contours    (array[][]) : Processed and filtered contours.
            {return_image=True} blank_image (cv::Mat)   : Blank image with drawn contours.
            {image is not None}     image       (cv::Mat)   : Passed image with drawn contours.
        """

        if return_image:
            blank_image = np.zeros(shape=self.image_shape, dtype=np.uint8)

            if image is not None:
                orig_img_cp = copy.deepcopy(image)

        contours, hierarchy = cv2.findContours(
            noise_removed_image, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE
        )

        cnt_fiter = self._contour_filtering(
            hierarchy, contours, contour_area_threshold, mode=1
        )
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
            try:
                cnt = using_contours[0][cnt_idx]
            except Exception:
                cnt = using_contours[cnt_idx]
            cnt_moments = cv2.moments(cnt)

            try:
                centroid_center_x = int(cnt_moments["m10"] / cnt_moments["m00"])
                centroid_center_y = int(cnt_moments["m01"] / cnt_moments["m00"])
            except ZeroDivisionError:
                return
            cnt_area = cnt_moments["m00"]

            if return_image:
                if image is not None:
                    cv2.drawContours(
                        orig_img_cp, using_contours, cnt_idx, (255, 0, 0), 2
                    )
                cv2.drawContours(blank_image, using_contours, cnt_idx, (255, 0, 0), 2)

            centroid_data.append((centroid_center_x, centroid_center_y, cnt_area))
            cnt_area_str = str(centroid_data[cnt_idx][2])

            if return_image and show_centers:
                cv2.circle(
                    blank_image,
                    (centroid_data[cnt_idx][0], centroid_data[cnt_idx][1]),
                    2,
                    (0, 255, 0),
                    2,
                )
                if image is not None:
                    cv2.circle(
                        orig_img_cp,
                        (centroid_data[cnt_idx][0], centroid_data[cnt_idx][1]),
                        2,
                        (0, 255, 0),
                        2,
                    )

            if return_image and show_areas:
                cv2.putText(
                    blank_image,
                    cnt_area_str,
                    (centroid_data[cnt_idx][0], centroid_data[cnt_idx][1]),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (255, 255, 255),
                    2,
                )
                if image is not None:
                    cv2.putText(
                        orig_img_cp,
                        cnt_area_str,
                        (centroid_data[cnt_idx][0], centroid_data[cnt_idx][1]),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1,
                        (1, 0, 0),
                        2,
                    )

        if return_image:
            if image is not None:
                return orig_img_cp, blank_image, using_contours
            else:
                return blank_image, using_contours
        else:
            return using_contours


class PointsProcessing(object):
    def __init__(self, len_of_integral_binary_resetter=5, icp_ref_points=None, *args, **kwargs):
        self.points_processing_image_shape = (720, 1280, 4)
        self.integral_diff_values_arr = []
        self.integral_diff_values_arr_len = len_of_integral_binary_resetter

        self.prev_closest_points = []
        self.prev_closest_point_dsts = []

        self.ref_points_icp_fitting_base = np.array([[449, 341], [845, 496], [690, 331]], dtype=int)
        self.ref_points_icp_fitting = np.array([[449, 341], [845, 496], [690, 331]], dtype=int)

        # print(icp_ref_points)
        if icp_ref_points is not None:
            self.ref_points_icp_fitting_base = icp_ref_points
            self.ref_points_icp_fitting = icp_ref_points

        super(PointsProcessing, self).__init__(*args, **kwargs)

    def _icp_fitting(self, ref_points, point_set, return_image=False, image=None):
        centroid_arr = point_set
        if return_image:
            blank_image = np.zeros(shape=self.image_shape, dtype=np.uint8)
            if image is not None:
                orig_img_cp = copy.deepcopy(image)

        _, icp_points = icp.icp(centroid_arr, ref_points, verbose=False)
        # points_int = np.rint(icp_points)

        if return_image:
            for pnt in icp_points:
                cv2.circle(blank_image, (int(pnt[0]), int(pnt[1])), 2, (0, 255, 0), 2)

                if image is not None:
                    cv2.circle(
                        orig_img_cp, (int(pnt[0]), int(pnt[1])), 2, (0, 255, 0), 2
                    )

        if return_image:
            if image is not None:
                return orig_img_cp, blank_image, icp_points
            else:
                return blank_image, icp_points
        else:
            return icp_points

    def _point_distances(self, point_arr1, point_arr2):
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

                dst2point = math.sqrt(
                    (abs(p_n[0] - p_k[0]) ** 2) + (abs(p_n[1] - p_k[1]) ** 2)
                )
                distance_table[p_n_idx].append(dst2point)

        return distance_table

    def _euclidian_closest_point(self, point_arr1, point_arr2):
        # Gives the closest point and the distance to the point from point a in array 1 to point b in array 2
        distance_table = self._point_distances(point_arr1, point_arr2)

        closest_point_idxes = []
        closest_points = []
        closest_point_dsts = []

        # for each reference point the closest point is written to the idx of the reference points list
        for dsts_2_ref_point in distance_table:
            closest_point_idx = min(
                range(len(dsts_2_ref_point)), key=dsts_2_ref_point.__getitem__
            )
            closest_point_idxes.append(closest_point_idx)
            closest_point_dsts.append(round(dsts_2_ref_point[closest_point_idx], 4))

        for point_idx in closest_point_idxes:
            closest_points.append(point_arr2[point_idx])

        return closest_points, closest_point_dsts

    def _duplicate_point_filter(self, closest_points, closest_point_dsts):
        # closest_points_np = np.rint(np.array([[2, 2], [3, 3], [4, 4], [3, 3], [2, 2], [1, 1]]))
        closest_points_np = np.rint(np.array(closest_points))

        # An index in indices is the same index in closest_points, and a value in indices is an index for a value in uniq_points, that is a value in closest_points with same index as indices
        uniq_points, indices = np.unique(closest_points_np, return_inverse=True, axis=0)

        num_closest_points = len(closest_points_np)
        num_uniq_closest_points = len(uniq_points)

        # Function stops here if
        if num_closest_points == num_uniq_closest_points:
            return closest_points, closest_point_dsts

        # >===================================> Duplicate point filter starts here
        # Checks which indices has a closest point as its closest point (finds duplicates in indices -> indices of indices)
        indices_of_indices = defaultdict(list)
        for i, item in enumerate(indices):
            indices_of_indices[item].append(i)
        indices_of_indices = {
            index_of_val_in_uniq_points: index_of_point_in_closest_points
            for index_of_val_in_uniq_points, index_of_point_in_closest_points in indices_of_indices.items()
            if len(index_of_point_in_closest_points) > 1
        }

        # Logic for changing the closest point x of a point b to its previous closest point y,
        # if there is another point c, which has point x as its closest point with smaller distance to it than point b
        for point_val_in_uniq_points, indices_of_p1 in indices_of_indices.items():
            comp_dsts = []
            for not_closest_point_idx in indices_of_p1:
                comp_dsts.append(closest_point_dsts[not_closest_point_idx])
            actual_closest_point_idx_in_indices_of_p1 = min(
                range(len(comp_dsts)), key=comp_dsts.__getitem__
            )
            actual_closest_point_idx = indices_of_p1[
                actual_closest_point_idx_in_indices_of_p1
            ]

            indices_of_p1.pop(actual_closest_point_idx_in_indices_of_p1)

            for not_closest_point_idx in indices_of_p1:
                closest_points[not_closest_point_idx] = self.prev_closest_points[
                    not_closest_point_idx
                ]
                closest_point_dsts[
                    not_closest_point_idx
                ] = self.prev_closest_point_dsts[not_closest_point_idx]

        return closest_points, closest_point_dsts

    def _point_thresholding(
        self,
        closest_points,
        closest_point_dsts,
        threshold,
        reset_reference_points_threshold,
    ):
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

            integral_check = (
                sum(self.integral_diff_values_arr) // self.integral_diff_values_arr_len
            )
            if integral_check > reset_reference_points_threshold:
                self.points_processing_reset()
        try:
            self.integral_diff_values_arr.append(max(diff_dsts))
            if len(self.integral_diff_values_arr) > self.integral_diff_values_arr_len:
                self.integral_diff_values_arr.pop(0)
        except ValueError:
            pass

        return pts_cp, pt_dsts_cp, diff_dsts

    def _reference_points_iteration(self, closest_points):
        self.ref_points_icp_fitting = np.array(closest_points, dtype=int)

    def fitted_point_filtering(self, point_arr1, point_arr2):
        closest_points, closest_point_dsts = self._euclidian_closest_point(
            point_arr1, point_arr2
        )

        (
            closest_points_filtered,
            closest_point_dsts_filtered,
        ) = self._duplicate_point_filter(closest_points, closest_point_dsts)

        (
            thresholded_closest_points,
            thresholded_closest_point_dsts,
            diff_dsts,
        ) = self._point_thresholding(
            closest_points_filtered,
            closest_point_dsts_filtered,
            threshold=50,
            reset_reference_points_threshold=100,
        )
        # Sometimes makes it better, sometimes not, sometimes worse
        self._reference_points_iteration(thresholded_closest_points)

        self.prev_closest_points = thresholded_closest_points
        self.prev_closest_point_dsts = thresholded_closest_point_dsts

        return thresholded_closest_points

    def i2rcp(self, rect_center_points, return_image=False, image=None):
        if return_image:
            blank_image = np.zeros(shape=self.points_processing_image_shape, dtype=np.uint8)
            if image is not None:
                img_cp = copy.deepcopy(image)

        icp_points = self._icp_fitting(
            self.ref_points_icp_fitting, rect_center_points, return_image=False
        )

        closest_points, closest_point_dsts = self._euclidian_closest_point(
            icp_points, rect_center_points
        )
        closest_points = self.fitted_point_filtering(icp_points, rect_center_points)

        if return_image:
            for pt in closest_points:
                cv2.circle(blank_image,(pt[0], pt[1]),2,(255, 0, 255),2,)
                if image is not None:
                    cv2.circle(img_cp,(pt[0], pt[1]),2,(255, 0, 255),2,)
        
        if return_image:
            if image is not None:
                return img_cp, blank_image, closest_points
            else:
                return blank_image, closest_points
        else:
            return closest_points

    def points_processing_reset(self):
        self.ref_points_icp_fitting = self.ref_points_icp_fitting_base
        self.prev_closest_points = []
        self.prev_closest_point_dsts = []
        self.integral_diff_values_arr = []


class ShapeProcessing(object):
    def __init__(self, *args, **kwargs):
        self.shape_processing_image_shape = (720, 1280, 4)
        super(ShapeProcessing, self).__init__(*args, **kwargs)

    def shape_fitting(self, contours, ratio_threshold, return_image=False, image=None):
        """Fit least area rectangles onto contours.

        Params:
            contours                (array[][]) : Contours that are to be fitted with the last area rectangles.
            ratio_threshold         (uint16)    : Ratio threshold between longest and shortest rectangle sides.
                                                  Rectangles below this ratio threshold are removed.
            image                   (cv::Mat)   : An image on which to draw fitted shapes.

        Returns:
                            fitted_boxes    (array[][4])    : Contour-fitted and filtered rectangles.
                            centroid_arr    (array[][2])    : Array of center points in each fitted rectangle.
        {return_image=True} blank_image     (cv::Mat)       : Blank image with drawn contours.
        {image != None}     image           (cv::Mat)       : Passed image with drawn contours.
        """
        if return_image:
            blank_image = np.zeros(shape=self.shape_processing_image_shape, dtype=np.uint8)
            if image is not None:
                orig_img_cp = copy.deepcopy(image)

        fitted_boxes = []

        # Had to change to iterate the first element of contours.....
        for cnt in contours[0]:
            rect = cv2.minAreaRect(cnt)

            rect_width = rect[1][0]
            rect_height = rect[1][1]

            rect_long = rect_height
            rect_short = rect_width

            if rect_height < rect_width:
                rect_long = rect_width
                rect_short = rect_height

            if rect_long > rect_short * ratio_threshold:
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                fitted_boxes.append(box)
                if return_image:
                    cv2.drawContours(blank_image, [box], 0, (0, 0, 255), 2)
                    if image is not None:
                        cv2.drawContours(orig_img_cp, [box], 0, (0, 0, 255), 2)

        centroid_arr = np.empty([len(fitted_boxes), 2], dtype=int)

        for cnt_idx in range(len(fitted_boxes)):
            cnt = fitted_boxes[cnt_idx]
            cnt_moments = cv2.moments(cnt)

            centroid_center_x = int(cnt_moments["m10"] / cnt_moments["m00"])
            centroid_center_y = int(cnt_moments["m01"] / cnt_moments["m00"])

            centroid_arr[cnt_idx] = [centroid_center_x, centroid_center_y]
        
        if return_image:
            if image is not None:
                return orig_img_cp, blank_image, fitted_boxes, centroid_arr
            else:
                return blank_image, fitted_boxes, centroid_arr
        else:
            return fitted_boxes, centroid_arr

    def convex_fitting(
        self,
        contours,
        convex_contours,
        area_diff_threshold,
        return_image=False,
        image=None,
    ):
        if return_image:
            blank_image = np.zeros(shape=self.image_shape, dtype=np.uint8)
            if image is not None:
                img_cp = copy.deepcopy(image)

        convexes_filtered = []
        for cnt_idx in range(len(contours)):
            cnt = contours[cnt_idx]
            cvx = convex_contours[cnt_idx]

            cvx_area = cv2.contourArea(cvx)
            cnt_area = cv2.contourArea(cnt)
            diff_area = cvx_area - cnt_area

            if diff_area < (cnt_area * area_diff_threshold):
                convexes_filtered.append(cvx)
                if return_image:
                    cv2.drawContours(
                        blank_image, convex_contours, cnt_idx, (0, 255, 0), 2
                    )
                    if image is not None:
                        cv2.drawContours(
                            img_cp, convex_contours, cnt_idx, (0, 255, 0), 2
                        )
        if return_image:
            if image is not None:
                return img_cp, blank_image, convexes_filtered
            else:
                return blank_image, convexes_filtered
        else:
            return convexes_filtered

    def line_fitting(
        self, contours, angle_threshold=50, return_image=False, image=None
    ):
        if return_image:
            blank_image = np.zeros(shape=self.image_shape, dtype=np.uint8)
            if image is not None:
                img_cp = copy.deepcopy(image)

        theta_set = set()
        for cnt in contours:
            rows, cols = blank_image.shape[:2]
            [vx, vy, x, y] = cv2.fitLine(cnt, cv2.DIST_L2, 0, 0.01, 0.01)
            theta = math.atan(vy / vx)
            theta = abs(theta * math.pi * 180) % 360
            theta_set.add(theta)
            lefty = int((-x * vy / vx) + y)
            righty = int(((cols - x) * vy / vx) + y)
            if return_image:
                cv2.line(blank_image, (cols - 1, righty), (0, lefty), (0, 255, 0), 4)
                cv2.putText(
                    blank_image,
                    str(round(theta, 1)) + " deg",
                    (x, y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (255, 255, 255),
                    2,
                )
                if image is not None:
                    cv2.line(img_cp, (cols - 1, righty), (0, lefty), (0, 255, 0), 4)
                    cv2.putText(
                        img_cp,
                        str(round(theta, 1)) + " deg",
                        (x, y),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1,
                        (255, 255, 255),
                        2,
                    )

        theta_arr = list(theta_set)
        parallell_line_count = 0
        for theta_outer in theta_arr:
            if 90 < theta_outer < 180:
                for theta_inner in theta_arr:
                    if (
                        (theta_outer + angle_threshold >= theta_inner)
                        and (theta_outer - angle_threshold <= theta_inner)
                        and (theta_outer != theta_inner)
                    ):
                        parallell_line_count += 1
            else:
                continue

        if return_image:
            if image is not None:
                return img_cp, blank_image, theta_arr, parallell_line_count
            else:
                return blank_image, theta_arr, parallell_line_count
        else:
            return theta_arr, parallell_line_count

    def corner_detection(self, line_fitted_img):
        """WIP"""
        line_fitted_img_cp = copy.deepcopy(line_fitted_img)
        # blur_line_fitted_img = cv2.GaussianBlur(line_fitted_img_cp, (5, 19), 5.2)

        blank_image = np.zeros(
            shape=[self.img_height, self.img_width, self.img_channels], dtype=np.uint8
        )
        blank_image_corners = np.zeros(
            shape=[self.img_height, self.img_width, self.img_channels], dtype=np.uint8
        )

        gray = cv2.cvtColor(line_fitted_img_cp, cv2.COLOR_BGR2GRAY)
        gray = np.float32(gray)
        dst = cv2.cornerHarris(gray, 13, 19, 0.04)

        # result is dilated for marking the corners, not important
        dst = cv2.dilate(dst, None)

        # Threshold for an optimal value, it may vary depending on the image.
        blank_image[dst > 0.00001 * dst.max()] = [0, 0, 255, 0]
        blank_image = cv2.cvtColor(blank_image, cv2.COLOR_BGR2GRAY)

        contours, hierarchy = cv2.findContours(
            blank_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
        )

        corner_point_arr = []
        for cnt in contours:
            cnt_moments = cv2.moments(cnt)

            centroid_center_x = int(cnt_moments["m10"] / cnt_moments["m00"])
            centroid_center_y = int(cnt_moments["m01"] / cnt_moments["m00"])

            cv2.circle(
                blank_image_corners,
                (centroid_center_x, centroid_center_y),
                2,
                (255, 0, 255),
                2,
            )

            corner_point_arr.append((centroid_center_x, centroid_center_y))

        return blank_image_corners, corner_point_arr

    def get_contour_from_rect(self, rect):
        return np.array(rect).reshape((-1, 1, 2)).astype(np.int32)

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

    def get_all_points_in_rects(
        self, rects, return_per_rect=False, return_image=False, image=None, decimation_lvl=100
    ):
        # Possible for rectangle-wise point extraction in this fnc (mv np.zeros blank img to rect in rects loop)
        if return_image:
            if image is not None:
                orig_img_cp = copy.deepcopy(image)

        blank_image = np.zeros(shape=self.image_shape, dtype=np.uint8)
        rects_arr_shape = np.shape(rects)
        num_of_rects = rects_arr_shape[0]

        # points_in_rects = []
        # for i in range(num_of_rects):
        #     points_in_rects.append([])

        if not return_per_rect:
            for rect in rects:
                ctr = self.get_contour_from_rect(rect)

                cv2.drawContours(
                    blank_image, [ctr], 0, (255, 255, 255), thickness=cv2.FILLED
                )
                if return_image and (image is not None):
                    cv2.drawContours(
                        orig_img_cp, [ctr], 0, (255, 255, 255), thickness=cv2.FILLED
                    )

            blank_image = cv2.cvtColor(blank_image, cv2.COLOR_BGR2GRAY)
            px_arr = np.argwhere(blank_image == 255)

        else:
            px_arr = []
            for rect in rects:
                ctr = self.get_contour_from_rect(rect)
                tmp_blank_image = np.zeros(shape=self.image_shape, dtype=np.uint8)

                cv2.drawContours(
                    tmp_blank_image, [ctr], 0, (255, 255, 255), thickness=cv2.FILLED
                )
                cv2.drawContours(
                    blank_image, [ctr], 0, (255, 255, 255), thickness=cv2.FILLED
                )
                if return_image and (image != None):
                    cv2.drawContours(
                        orig_img_cp, [ctr], 0, (255, 255, 255), thickness=cv2.FILLED
                    )

                tmp_blank_image = cv2.cvtColor(tmp_blank_image, cv2.COLOR_BGR2GRAY)
                blank_image = cv2.cvtColor(blank_image, cv2.COLOR_BGR2GRAY)
                tmp_px_arr = np.argwhere(tmp_blank_image == 255)
                px_arr.append(tmp_px_arr)
        
        if decimation_lvl is not None:
            decimated_px_arr = px_arr[::decimation_lvl].copy()

        if return_image:
            if image is not None:
                return orig_img_cp, blank_image, decimated_px_arr
            else:
                return blank_image, decimated_px_arr
        else:
            return px_arr

    def rect_filtering(
        self, i2rcp_points, fitted_boxes, return_rectangles_separately=False, return_image=False, image=None
    ):
        if return_image:
            blank_image = np.zeros(shape=self.shape_processing_image_shape, dtype=np.uint8)
            if image is not None:
                img_cp = copy.deepcopy(image)

        relevant_rects = self.get_relevant_rects(i2rcp_points, fitted_boxes)
        pointed_rects_img, _, points_in_rects = self.get_all_points_in_rects(relevant_rects, return_per_rect=return_rectangles_separately, return_image=True, image=image)
        self.pointed_rects_img = pointed_rects_img

        if return_image:
            for pnt in i2rcp_points:
                cv2.circle(blank_image, (int(pnt[0]), int(pnt[1])), 5, (255, 0, 255), 2)
                if image is not None:
                    cv2.circle(img_cp, (int(pnt[0]), int(pnt[1])), 5, (255, 0, 255), 2)

            for box in relevant_rects:
                box = np.int0(box)
                cv2.drawContours(blank_image, [box], 0, (0, 0, 255), 2)
                if image is not None:
                    cv2.drawContours(img_cp, [box], 0, (0, 0, 255), 2)

        if return_image:
            if image is not None:
                return img_cp, blank_image, relevant_rects, points_in_rects
            else:
                return blank_image, relevant_rects, points_in_rects
        else:
            return relevant_rects, points_in_rects


class FeatureDetection(ImageFeatureProcessing, PointsProcessing, ShapeProcessing):
    """Algorithms for feature-processing-based object detection."""

    def __init__(self, image_shape, len_of_integral_binary_resetter=5, icp_ref_points=None):
        super(FeatureDetection, self).__init__(image_shape=image_shape, len_of_integral_binary_resetter=len_of_integral_binary_resetter, icp_ref_points=icp_ref_points)

        self.image_shape = image_shape
        self.shape_processing_image_shape = self.image_shape
        self.points_processing_image_shape = self.image_shape

        self.ref_points_icp_fitting_base = np.array([[449, 341], [845, 496], [690, 331]], dtype=int)
        self.ref_points_icp_fitting = np.array([[449, 341], [845, 496], [690, 331]], dtype=int)

        if icp_ref_points is not None:
            self.ref_points_icp_fitting_base = icp_ref_points
            self.ref_points_icp_fitting = icp_ref_points

        # Classification
        self.detection = False

    def feature_detection(self, original_image, hsv_params, noise_removal_params):
        _, hsv_mask, hsv_validation_img = self.hsv_processor(original_image, *hsv_params)
        self.hsv_validation_img = hsv_validation_img
        
        try:
            nr_img = self.noise_removal_processor(hsv_mask, *noise_removal_params)
            self.nr_img = nr_img
        except Exception:
            pass

        try:
            using_cnts = self.contour_processing(nr_img, 300, return_image=False)
        except Exception:
            pass
        
        try:
            shape_img, _, fitted_boxes, centroid_arr = self.shape_fitting(using_cnts, 5, return_image=True, image=original_image)
            self.shape_img = shape_img
        except Exception:
            pass

        try:
            i2rcp_image_blank, i2rcp_points = self.i2rcp(centroid_arr, return_image=True, image=None)
            self.i2rcp_image_blank = i2rcp_image_blank
        except Exception:
            pass

        # Changed returns to obj vars
        try:
            rect_flt_img, _, self.relevant_rects, self.points_in_rects = self.rect_filtering(i2rcp_points, fitted_boxes, return_rectangles_separately=False, return_image=True, image=original_image)
            self.rect_flt_img = rect_flt_img
            self.pointed_rects_img = self.pointed_rects_img
        except Exception:
            pass

        return self.relevant_rects, self.points_in_rects

    def bounding_box_processor(
        self, all_points_in_rects, label_name, return_image=False, image=None
    ):
        if return_image:
            blank_image = np.zeros(shape=self.image_shape, dtype=np.uint8)
            if image is not None:
                img_cp = copy.deepcopy(image)

        x_lst, y_lst = zip(*all_points_in_rects)

        xmin = min(x_lst)
        ymin = min(y_lst)
        xmax = max(x_lst)
        ymax = max(y_lst)

        bbox_area = (ymax - ymin) * (xmax - xmin)
        bbox_points = (xmin, ymin, xmax, ymax)

        if return_image:
            cv2.rectangle(blank_image, (ymin, xmin), (ymax, xmax), (0, 255, 0), 2)
            cv2.putText(
                blank_image,
                label_name,
                (ymin, xmin - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (36, 255, 12),
                2,
            )
            if image is not None:
                cv2.rectangle(img_cp, (ymin, xmin), (ymax, xmax), (0, 255, 0), 2)
                cv2.putText(
                    img_cp,
                    label_name,
                    (ymin, xmin - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.9,
                    (36, 255, 12),
                    2,
                )

        # insert 'if self.classified_obj' check here
        self.bounding_box_corners = bbox_points
        self.bounding_box_area = bbox_area
        if return_image:
            if image is not None:
                self.bounding_box_image = img_cp
                self.bounding_box_image_blank = blank_image
                return img_cp, blank_image, bbox_points, bbox_area
            else:
                self.bounding_box_image_blank = blank_image
                return blank_image, bbox_points, bbox_area
        else:
            return bbox_points, bbox_area

    def classification(
        self, original_image, label_name, hsv_params, noise_removal_params
    ):

        try:
            self.relevant_rects, self.points_in_rects = self.feature_detection(original_image, hsv_params, noise_removal_params)
        except Exception:
            pass
        
        try:
            line_fitting_img, _, theta_arr, self.parallell_line_count = self.line_fitting(self.relevant_rects, angle_threshold=50, return_image=True, image=original_image)
            self.line_fitting_img = line_fitting_img
        except Exception:
            pass


        try:
            detection = False
            if self.parallell_line_count > 1:
                detection = True
                self.bbox_img, _, self.bbox_points, self.bbox_area = self.bounding_box_processor(
                    self.points_in_rects, label_name, return_image=True, image=original_image)

            self.detection = detection
        except Exception:
            pass
        return self.bbox_points, self.bbox_area, self.points_in_rects, self.detection
