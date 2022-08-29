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
    """Methods for processing of (matrix-like) image data.
    The methods here serve a variety of purposes such as colour filtering, binary data processing, and contour extraction.
    Needs instantiation with a (matrix) shape of the image that going to be manipulated.

    Params:
        image_shape (array-like [3]): Shape of the relevant image - height, width, channels.
    
    Attributes:
        image_shape (array-like [3]): Same as in the parameters. Is passed down to sub-classes.

    Methods:
        hsv_processor:              Takes a raw image and applies Hue-Saturation-Value filtering
        noise_removal_processor:    Applies various noise removal and morphism algorithms.
        contour_filtering:          Filters contours according to contour hierarchy and area.
        contour_processing:         Finds contours in a pre-processed image and filters them.
    """
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
            morphised_image (cv::Mat): Passed HSV image with morphised features using blur, thresholding, erosion, dilation.
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
        morphised_image = cv2.dilate(
            erosion_img, erosion_dilation_kernel, iterations=dilation_iterations
        )

        return morphised_image

    def contour_filtering(
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
            contour_len_threshold   (uint16)    : {default=20} Threshold for filtering based on length of a contour.
            mode                    (uint8)     : {default=1} Mode for hierarchical filtering.
                                                  Mode 1 leaves only the contours that do not have any hierarchical children.
                                                  Mode 2 leaves the contours that do not have any hierarchical children or neighbours.

        Returns:
            filtered_contours (array[][]): Filtered contours.
        """
        filtered_contours = []

        # Returns if there are no contours
        try:
            num_of_contours = len(contours)
        except TypeError:
            return

        try:
            for cnt_idx in range(num_of_contours):
                cnt_hier = hierarchy[0][cnt_idx]

                if mode == 1:
                    # Gets contours that are of lowest hierarchical class, but can have neighbours
                    if (
                        ((cnt_hier[0] == cnt_idx + 1) or (cnt_hier[0] == -1))
                        and ((cnt_hier[1] == cnt_idx - 1) or (cnt_hier[1] == -1))
                        and (cnt_hier[2] == -1)
                    ):
                        cnt = contours[cnt_idx]
                        cnt_area = cv2.contourArea(cnt)
                        
                        # Filters out contours with less-than-predefined threshold area 
                        if cnt_area < contour_area_threshold:
                            filtered_contours.append(False)
                        else:
                            # Filters out contours with less-than-predefined threshold perimeter
                            if len(cnt) > contour_len_threshold:
                                filtered_contours.append(True)
                            else:
                                filtered_contours.append(False)
                    else:
                        filtered_contours.append(False)

                if mode == 2:
                    # Gets contours that are of lowest hierarchical class and without neighbours
                    if (len([i for i, j in zip(cnt_hier, [-1, -1, -1, cnt_idx - 1])if i == j])!= 4):
                        cnt = contours[cnt_idx]
                        cnt_area = cv2.contourArea(cnt)
                        # Filters out contours with less-than-predefined threshold area 
                        if cnt_area < contour_area_threshold:
                            filtered_contours.append(False)
                        else:
                            # Filters out contours with less-than-predefined threshold perimeter
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
            enable_convex_hull      (bool)      : {default=False} Enable convex hull contour approximation method.
            return_image            (bool)      : {default=True} False to return only contour data.
                                                  If param 'image' is None - returns a blanked image with drawn contour data.
                                                  If param 'image' is an image - returns both drawn blanked and passed images.
            image                   (cv::Mat)   : {default=None} An image on which to draw processed contours.
            show_centers            (bool)      : {default=True} Draw contour centers in the returned image(s).
            show_areas              (bool)      : {default=False} Draw contour areas in the returned image(s).

        Returns:
                                contour_arr (array[][]) : Processed and filtered contours.
            {return_image=True} blank_image (cv::Mat)   : Blank image with drawn contours.
            {image is not None} img_cp      (cv::Mat)   : Passed image with drawn contours.
        """

        if return_image:
            blank_image = np.zeros(shape=self.image_shape, dtype=np.uint8)

            if image is not None:
                img_cp = copy.deepcopy(image)

        contours, hierarchy = cv2.findContours(
            noise_removed_image, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE
        )

        cnt_filter = self.contour_filtering(
            hierarchy, contours, contour_area_threshold, mode=1
        )
        contours_array = np.array(contours)
        contours_filtered = contours_array[cnt_filter]
        
        # Container list
        contour_arr = []

        # Applies convex hull contour approximation if specified in the parameters
        if enable_convex_hull:
            hull_array = []
            for cnt_idx in range(len(contours_filtered)):
                hull_array.append(cv2.convexHull(contours_filtered[cnt_idx], False))
            contour_arr = hull_array
        else:
            contour_arr = contours_filtered

        # Contour centers
        centroid_data = []
        for cnt_idx in range(len(contours_filtered)):
            try:
                cnt = contour_arr[0][cnt_idx]
            except Exception:
                cnt = contour_arr[cnt_idx]
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
                        img_cp, contour_arr, cnt_idx, (255, 0, 0), 2
                    )
                cv2.drawContours(blank_image, contour_arr, cnt_idx, (255, 0, 0), 2)

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
                        img_cp,
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
                        img_cp,
                        cnt_area_str,
                        (centroid_data[cnt_idx][0], centroid_data[cnt_idx][1]),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1,
                        (1, 0, 0),
                        2,
                    )

        if return_image:
            if image is not None:
                return img_cp, blank_image, contour_arr
            else:
                return blank_image, contour_arr
        else:
            return contour_arr


class PointsProcessing(object):
    """Methods for processing, filtering, fitting, and otherwise manipulating of 2D point-type data.
    The main purpose of this class is to provide point-based detection and classification algorithms for a 2D environment.

    Params:
        len_of_integral_binary_resetter (uint8)             : {default=5} Number of steps in the binary integral reference point resetter.
        icp_ref_points                  (A[N][2, uint16])   : {default=[[449, 341], [845, 496], [690, 331]]} Initial reference points for the point fitting algorithm.

    Attributes:
        points_processing_image_shape   (array-like [3])                            : Shape of the image to be drawn with point visualization - height, width, channels.
        integral_diff_values_arr        (A[len_of_integral_binary_resetter, uint16]): List of delta distances between delta times used for the binary integral resetter.
        integral_diff_values_arr_len    (uint8)                                     : Member variable copy of the 'len_of_integral_binary_resetter' parameter.
        prev_closest_points             (A[len(icp_ref_points)][2, uint8])          : List of closest points from the previous timestep with length N according to how many reference points there are.
        prev_closest_points_dsts        (A[len(icp_ref_points)][float16])           : List of closest point distances from the previous timestep.
        ref_points_icp_fitting_base     (A[N][2, uint16])                           : Member variable copy of the 'icp_ref_points' parameter. This variable stays static for the duration of the runtime.
                                                                                      Is used as a fallback reference using the binary integral resetter.
        ref_points_icp_fitting          (A[N][2, uint16])                           : Dynamic reference points list. Used as argument in the ICP, recursively iterated using the I2RCP. 

    Methods:
        icp_fitting                 : Applies iterative closest points (ICP) algorithm given a set of N reference points and a set of M points to be fitted, where N<=M.  
        point_distances             : Finds distances from every point in array A to every point in array B.
        euclidian_closest_point     : (ECD) Brute-force algorithm to find the closest point and its euclidian distance for every point in array A in comparison to every point in array B.
        duplicate_point_filter      : (DPF) Finds if there are multiple points in set A that have the same point in set B as their closest points. 
                                      The point 'a' with the smallest distance will be left with their actual closest point, whilst the other(s) will have their previous closest point(s).
        point_thresholding          : Binary Integral Derivative (BID) controller. Checks for point position deltas - if delta is too huge, point will take its previous step value.
                                      Gathers integral sum of position deltas over N previous steps - if the sum delta is too huge, resets the initial reference points.
        reference_points_iteration  : Iterates the reference points array with new values.
        fitted_point_filtering      : (FPF) Wrapper function for ECD, DPF, and BID. Delays the closest points and distances by one step.
        i2rcp                       : Intra-Iterative Recursive Closest Point (I2RCP) - an algorithm for optimal point fitting using ICP as the base, but also applying ECD, DPF, and BID serially.
                                      The algorithm: I2RCP := ICP -> ECD -> DPF -> BID -> ref. pts. iteration -> delay (z^-1) -> loop (ICP)
        points_processing_reset     : Resets the current reference points to the base (initial) reference points.
    """
    def __init__(self, len_of_integral_binary_resetter=5, icp_ref_points=None, *args, **kwargs):
        self.points_processing_image_shape = (720, 1280, 4) # Only used for drawing data on image. Gets adaptively overwritten by init argument from FeatureDetection class.
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

    def icp_fitting(self, ref_points, points_set, return_image=False, image=None):
        """Applies iterative closest points (ICP) algorithm given a set of N reference points and a set of M points to be fitted, where N<=M.

        Params:
            ref_points      (A[N][2, uint8])    : Reference points to be fitted to the points' set.
            points_set      (A[M][2, uint8])    : A set of points on which the reference points will be fitted (N <= M).
            return_image    (bool)              : {default=False} False to return only point data.
                                                  If param 'image' is None - returns a blanked image with drawn point data.
                                                  If param 'image' is an image - returns both drawn blanked and passed images.
            image           (cv::Mat)           : An image on which to draw the processed points.

        Returns:
                                icp_points  (array[][]) : Processed and fitted points.
            {return_image=True} blank_image (cv::Mat)   : Blank image with drawn points.
            {image is not None} img_cp      (cv::Mat)   : Passed image with drawn points.
        """

        if return_image:
            blank_image = np.zeros(shape=self.image_shape, dtype=np.uint8)
            if image is not None:
                img_cp = copy.deepcopy(image)

        _, icp_points = icp.icp(points_set, ref_points, verbose=False)
        # points_int = np.rint(icp_points)

        if return_image:
            for pnt in icp_points:
                cv2.circle(blank_image, (int(pnt[0]), int(pnt[1])), 2, (0, 255, 0), 2)

                if image is not None:
                    cv2.circle(
                        img_cp, (int(pnt[0]), int(pnt[1])), 2, (0, 255, 0), 2
                    )

        if return_image:
            if image is not None:
                return img_cp, blank_image, icp_points
            else:
                return blank_image, icp_points
        else:
            return icp_points

    def point_distances(self, point_arr1, point_arr2):
        """Find distance from every reference point (array 1) to every point in array 2.
        Stores values in table formatted by: array A of len(array_1),
        where every element is array B of len(array_2. Elements in arr B are distances from point indexed in A according to array 1 to a point indexed in B according to array 2.

        Params:
            point_arr1  (A[N][2, uint8])    : Reference points.
            point_arr2  (A[M][2, uint8])    : The set of points to which distances are going to be found.

        Returns:
            distance_table  (A[N][M, float16]) : Processed and fitted points.
        """

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

    def euclidian_closest_point(self, point_arr1, point_arr2):
        """(ECD) Brute-force algorithm to find the closest point and its euclidian distance for every point in array A in comparison to every point in array B.

        Params:
            point_arr1  (A[N][2, uint8])    : Reference points.
            point_arr2  (A[M][2, uint8])    : The set of points to which distances are going to be found.

        Returns:
            closest_points      (A[N][2, uint16])      : For index 'a' in point_arr1, closest point coordinates 'x' and 'y' picked from point_arr2.
            closest_point_dts   (A[N, float16])        : For index 'a' in point_arr1, distances from the reference point in point_arr1 to its closest point in point_arr2.
        """

        distance_table = self.point_distances(point_arr1, point_arr2)

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

    def duplicate_point_filter(self, closest_points, closest_point_dsts):
        """(DPF) Finds if there are multiple points in set A that have the same point in set B as their closest points. 
        The point 'a' with the smallest distance will be left with their actual closest point, whilst the other(s) will have their previous closest point(s).

        Params:
            closest_points      (A[N][2, uint16])      : For index 'a' in set of points A, closest point coordinates 'x' and 'y' picked from set of points B.
            closest_point_dts   (A[N, float16])        : For index 'a' in set of points A, distances from the reference point in set of points A to its closest point in set of points B.

        Returns:
            closest_points      (A[N][2, uint16])      : Same struct as the param, but filtered of duplicates.
            closest_point_dts   (A[N, float16])        : Same struct as the param, but filtered of duplicates.
        """

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

    def point_thresholding(
        self,
        closest_points,
        closest_point_dsts,
        threshold,
        reset_reference_points_threshold,
    ):
        """Binary Integral Derivative (BID) controller. Checks for point position deltas - if delta is too huge, point will take its previous step value.
        Gathers integral sum of position deltas over N previous steps - if the sum delta is too huge, resets the initial reference points.
        
        Params:
            closest_points                      (A[N][2, uint16])   : For index 'a' in set of points A, closest point coordinates 'x' and 'y' picked from set of points B.
            closest_point_dts                   (A[N, float16])     : For index 'a' in set of points A, distances from the reference point in set of points A to its closest point in set of points B.
            threshold                           (uint32)            : The value at which the derivative-term filter disregards change in point coordinates.
            reset_reference_points_threshold    (float32)           : Max sum of travel differences, at which point the reference points for the I2C will be reset.

        Returns:
            pts_cp      (A[N][2, uint16])      : Same struct as the closest_points, but processed.
            pt_dsts_cp  (A[N, float16])        : Same struct as the closest_point_dts, but processed.
            diff_dsts   (A[N, float16])        : Travel distance deltas for each closest point.
        """
        pts_cp = copy.deepcopy(closest_points)
        pt_dsts_cp = copy.deepcopy(closest_point_dsts)

        diff_dsts = []
        for i in range(len(self.prev_closest_point_dsts)):
            closest_pt_dst = pt_dsts_cp[i]
            prev_closest_pt_dst = self.prev_closest_point_dsts[i]

            # Calculate travel ditance deltas
            diff_prev_current_dst = abs(prev_closest_pt_dst - closest_pt_dst)
            diff_dsts.append(diff_prev_current_dst)

            # Delta thresholding
            if diff_prev_current_dst > threshold:
                pts_cp[i] = self.prev_closest_points[i]
                pt_dsts_cp[i] = self.prev_closest_point_dsts[i]

            # Checks if the ref points should be reset
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

    def reference_points_iteration(self, closest_points):
        """Iterates the reference points member attribute array with new values.

        Params:
            closest_points (A[N][2, uint16]): Should be in the format of: for index 'a' in set of points A, closest point coordinates 'x' and 'y' picked from set of points B.
        """
        self.ref_points_icp_fitting = np.array(closest_points, dtype=int)

    def fitted_point_filtering(self, point_arr1, point_arr2):
        """(FPF) Wrapper function for ECD, DPF, and BID. Delays the closest points and distances by one step.

        Params:
            point_arr1  (A[N][2, uint8]): Reference points.
            point_arr2  (A[M][2, uint8]): The set of points on which the reference points are going to be fitted.

        Returns:
            thresholded_closest_points (A[N][2, uint16]): Fitted points array.
        """
        closest_points, closest_point_dsts = self.euclidian_closest_point(
            point_arr1, point_arr2
        )

        (
            closest_points_filtered,
            closest_point_dsts_filtered,
        ) = self.duplicate_point_filter(closest_points, closest_point_dsts)

        (
            thresholded_closest_points,
            thresholded_closest_point_dsts,
            diff_dsts,
        ) = self.point_thresholding(
            closest_points_filtered,
            closest_point_dsts_filtered,
            threshold=50,
            reset_reference_points_threshold=100,
        )

        self.reference_points_iteration(thresholded_closest_points)

        self.prev_closest_points = thresholded_closest_points
        self.prev_closest_point_dsts = thresholded_closest_point_dsts

        return thresholded_closest_points

    def i2rcp(self, rect_center_points, return_image=False, image=None):
        """Intra-Iterative Recursive Closest Point (I2RCP) - an algorithm for optimal point fitting using ICP as the base, but also applying ECD, DPF, and BID serially.
        The algorithm: I2RCP := ICP -> ECD -> DPF -> BID -> ref. pts. iteration -> delay (z^-1) -||> loop (ICP)

        Params:
            rect_center_points  (A[N][2, uint16])   : Set of points on which the reference points going to be fitted.
            return_image        (bool)              : {default=False} False to return only point data.
                                                      If param 'image' is None - returns a blanked image with drawn point data.
                                                      If param 'image' is an image - returns both drawn blanked and passed images.
            image               (cv::Mat)           : An image on which to draw the processed points.
   

        Returns:
            closest_points                  (array[][]) : I2RCP fitted point array.
            {return_image=True} blank_image (cv::Mat)   : Blank image with drawn points.
            {image is not None} img_cp      (cv::Mat)   : Passed image with drawn points.
        """
        if return_image:
            blank_image = np.zeros(shape=self.points_processing_image_shape, dtype=np.uint8)
            if image is not None:
                img_cp = copy.deepcopy(image)

        icp_points = self.icp_fitting(
            self.ref_points_icp_fitting, rect_center_points, return_image=False
        )

        closest_points, closest_point_dsts = self.euclidian_closest_point(
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
        """Resets the current reference points to the base (initial) reference points.
        Affects only member attributes.
        """
        self.ref_points_icp_fitting = self.ref_points_icp_fitting_base
        self.prev_closest_points = []
        self.prev_closest_point_dsts = []
        self.integral_diff_values_arr = []


class ShapeProcessing(object):
    """Processing of 2D shapes and contours in the form of array-like objects. 

    Methods:
        shape_fitting           : Fit least area rectangles onto contours.
        line_fitting            : Fit lines onto countours through their centers going alongside moment direction.
        corner_detection        : Fit corners onto a line fitted image with blank background.
        get_contour_from_rect   : Reshape a rectangle which is defined by 4 corners into a contour.
        does_ctr_contain_point  : Check if a point is contained within the boundaries of a contour.
        get_relevant_rects      : Return rectangles which have at least one point from the parameter array within their boundaries.
        get_all_points_in_rects : Return all of the point coordinates within boundaries of rectangles. Can return element-wise decimated coordinate array.
        rect_filtering          : Return pixel values of contained within the rectangles which encompass at least one of the points in the parameter point array.
    """
    def __init__(self, *args, **kwargs):
        self.shape_processing_image_shape = (720, 1280, 4)
        super(ShapeProcessing, self).__init__(*args, **kwargs)

    def shape_fitting(self, contours, ratio_threshold, return_image=False, image=None):
        """Fit least area rectangles onto contours.

        Params:
            contours        (array[][]) : Contours that are to be fitted with the last area rectangles.
            ratio_threshold (uint16)    : Ratio threshold between longest and shortest rectangle sides.
                                          Rectangles below this ratio threshold are removed.
            return_image    (bool)      : {default=False} False to return only the shape data.
                                          If param 'image' is None - returns a blanked image with drawn shape data.
                                          If param 'image' is an image - returns both drawn blanked and passed images.
            image           (cv::Mat)   : An image on which to draw fitted shapes.

        Returns:
                            fitted_boxes    (array[][4])    : Contour-fitted and filtered rectangles.
                            centroid_arr    (array[][2])    : Array of center points in each fitted rectangle.
        {return_image=True} blank_image     (cv::Mat)       : Blank image with drawn rectangles.
        {image != None}     img_cp          (cv::Mat)       : Passed image with drawn rectangles.
        """
        if return_image:
            blank_image = np.zeros(shape=self.shape_processing_image_shape, dtype=np.uint8)
            if image is not None:
                img_cp = copy.deepcopy(image)

        fitted_boxes = []

        # Iterate over the contours
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
                        cv2.drawContours(img_cp, [box], 0, (0, 0, 255), 2)

        centroid_arr = np.empty([len(fitted_boxes), 2], dtype=int)

        for cnt_idx in range(len(fitted_boxes)):
            cnt = fitted_boxes[cnt_idx]
            cnt_moments = cv2.moments(cnt)

            centroid_center_x = int(cnt_moments["m10"] / cnt_moments["m00"])
            centroid_center_y = int(cnt_moments["m01"] / cnt_moments["m00"])

            centroid_arr[cnt_idx] = [centroid_center_x, centroid_center_y]
        
        if return_image:
            if image is not None:
                return img_cp, blank_image, fitted_boxes, centroid_arr
            else:
                return blank_image, fitted_boxes, centroid_arr
        else:
            return fitted_boxes, centroid_arr

    def line_fitting(
        self, contours, angle_threshold=50, return_image=False, image=None
    ):
        """Fit lines onto countours through their centers going alongside moment direction.

        Params:
            contours        (array[][]) : Contours that are to be fitted with the least area rectangles.
            angle_threshold (uint16)    : {default=50} Max angle (deg) between a line and the image frame horizontally.
                                          Lines above this threshold are filtered out.
            return_image    (bool)      : {default=False} False to return only the line data.
                                          If param 'image' is None - returns a blanked image with drawn line data.
                                          If param 'image' is an image - returns both drawn blanked and passed images.
            image           (cv::Mat)   : An image on which to draw fitted lines.

        Returns:
                            parallel_line_count (uint8)         : Number of fitted lines.
                            theta_arr           (float32)       : Array of fitted line angles (deg).
        {return_image=True} blank_image         (cv::Mat)       : Blank image with drawn lines.
        {image != None}     img_cp              (cv::Mat)       : Passed image with drawn lines.
        """
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
        """Fit corners onto a line fitted image with blank background.

        Params:
            line_fitted_img (cv::Mat): A 2D matrix with drawn lines in a distinct colour and zeroed background.

        Returns:
            blank_image_corners (cv::Mat)           : A 2D matrix with fitted corners and blank background.
            corner_point_arr    (A[N][2, uint32])   : An array of 2D points in the image that mark the coordinates of the corners.
        """
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
        """Reshape a rectangle which is defined by 4 corners into a contour.

        Params:
            rect (A[4][2, uint32]): An array of 4 points which defines a rectangle.

        Returns:
            contour (A[N][2, uint32]): A contour of the recatngle in the form of a vector of points.
        """
        return np.array(rect).reshape((-1, 1, 2)).astype(np.int32)

    def does_ctr_contain_point(self, ctr, point):
        """Check if a point is contained within the boundaries of a contour.

        Params:
            ctr     (A[N][2, uint32])       : A contour in the form of a vector of points.
            point   (array-like[2, uint32]) : A 2D point.
        
        Returns:
                    (bool)                  : Returns True if the point is within the contour.
        """
        indicator = cv2.pointPolygonTest(ctr, tuple(point), measureDist=False)
        if indicator >= 0:
            return True
        else:
            return False

    def get_relevant_rects(self, point_arr, rect_arr):
        """Return rectangles which have at least one point from the parameter array within their boundaries.

        Params:
            point_arr   (A[N][2, uint32])   : An array of 2D points.
            rect_arr    (A[M][4][2, uint32]): An array of 4 2D points, each of which define a rectangle.
        
        Returns:
            relevant_rects (A[M][4][2, uint32]): Array of filtered rectangles.
        """
        relevant_rects = []
        for rect in rect_arr:
            ctr = self.get_contour_from_rect(rect)
            for point in point_arr:
                is_in_rect = self.does_ctr_contain_point(ctr, point)
                if is_in_rect:
                    relevant_rects.append(rect)

        return relevant_rects

    def get_all_points_in_rects(
        self, rects, decimation_lvl=100, return_per_rect=False, return_image=False, image=None
    ):
        """Return all of the point coordinates within boundaries of rectangles. Can return element-wise decimated coordinate array.

        Params:
            rects           (A[M][4][2, uint32]): An array of 4 2D points, each of which define a rectangle.
            decimation_lvl  (uint32)            : {default=100} Reduces the size of the point array by doing element-wise decimation.
            return_per_rect (bool)              : {default=False} If set to True, returns one array per rectangle, instead of a single array.
            return_image    (bool)              : {default=False} False to return only the point data.
                                                  If param 'image' is None - returns a blanked image with drawn point data.
                                                  If param 'image' is an image - returns both drawn blanked and passed images.
            image           (cv::Mat)           : An image on which to draw points.
        Returns:
                            px_arr      (A[N][2, uint32])   : An array of 2D points.
        {return_image=True} blank_image (cv::Mat)           : Blank image with drawn points.
        {image != None}     img_cp      (cv::Mat)           : Passed image with drawn points.
        """
        if return_image:
            if image is not None:
                img_cp = copy.deepcopy(image)

        blank_image = np.zeros(shape=self.image_shape, dtype=np.uint8)
        rects_arr_shape = np.shape(rects)
        num_of_rects = rects_arr_shape[0]

        if not return_per_rect:
            for rect in rects:
                ctr = self.get_contour_from_rect(rect)

                cv2.drawContours(
                    blank_image, [ctr], 0, (255, 255, 255), thickness=cv2.FILLED
                )
                if return_image and (image is not None):
                    cv2.drawContours(
                        img_cp, [ctr], 0, (255, 255, 255), thickness=cv2.FILLED
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
                        img_cp, [ctr], 0, (255, 255, 255), thickness=cv2.FILLED
                    )

                tmp_blank_image = cv2.cvtColor(tmp_blank_image, cv2.COLOR_BGR2GRAY)
                blank_image = cv2.cvtColor(blank_image, cv2.COLOR_BGR2GRAY)
                tmp_px_arr = np.argwhere(tmp_blank_image == 255)
                px_arr.append(tmp_px_arr)
        
        if decimation_lvl is not None:
            decimated_px_arr = px_arr[::decimation_lvl].copy()

        if return_image:
            if image is not None:
                return img_cp, blank_image, decimated_px_arr
            else:
                return blank_image, decimated_px_arr
        else:
            return px_arr

    def rect_filtering(
        self, i2rcp_points, fitted_boxes, return_rectangles_separately=False, return_image=False, image=None
    ):
        """Return pixel values of contained within the rectangles which encompass at least one of the points in the parameter point array.

        Params:
            i2rcp_points                    (A[N][2, uint32])   : A set of I2RCP fitted points.
            fitted_boxes                    (A[M][4][2, uint32]): An array of 4 2D points, each of which define a rectangle.
            return_rectangles_separately    (bool)              : {default=False} If set to True, returns one array per rectangle, instead of a single array.
            return_image                    (bool)              : {default=False} False to return only the point data.
                                                                  If param 'image' is None - returns a blanked image with drawn point data.
                                                                  If param 'image' is an image - returns both drawn blanked and passed images.
            image                           (cv::Mat)           : An image on which to draw points.
        Returns:
                            relevant_rects  (A[M][4][2, uint32]): Array of filtered rectangles.
                            points_in_rects (A[N][2, uint32])   : An array of 2D points.
        {return_image=True} blank_image     (cv::Mat)           : Blank image with drawn points.
        {image != None}     img_cp          (cv::Mat)           : Passed image with drawn points.
        """
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
    """Algorithms for feature-processing-based object detection and classification."""

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
        """Feature detection pipeline.
        Applies HSV (colour) filter, morphism of canny image, contour fitting and filtering, shape fitting, I2RCP, and rectangle filtering.

        Params:
            original_image          (cv::Mat)               : BGRA image.
            hsv_params              (array-like[6, uint8])  : HSV params. See 'hsv_processor' function.
            noise_removal_params    (array-like[9, float32]): Noise removal and morpism params. See 'noise_removal_processor' function.

        Returns:
            relevant_rects  (A[M][4][2, uint32]): Array of filtered rectangles.
            points_in_rects (A[N][2, uint32])   : An array of 2D points.
        """
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
        """Generates a rectangular bounding box with a label.

        Params:
            all_points_in_rects (A[N][2, uint32])   : An array of 2D points.
            label_name          (string)            : Name of the detected object.
            return_image        (bool)              : {default=False} False to return only the point data.
                                                      If param 'image' is None - returns a blanked image with drawn bounding box.
                                                      If param 'image' is an image - returns both drawn blanked and passed images.
            image               (cv::Mat)           : An image on which to draw bounding box.
        Returns:
                            bbox_area   (float32)           : Area of the bounding box region in the image.
                            bbox_points (A[4][2, uint32])   : An array of 4 2D points which indicate the bounding box corners.
        {return_image=True} blank_image (cv::Mat)           : Blank image with drawn bounding box.
        {image != None}     img_cp      (cv::Mat)           : Passed image with drawn bounding box.
        """
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
        """Classifies an object in accordance to given label name and line fitting.
        Params:
            original_image          (cv::Mat)               : BGRA image.
            label_name              (string)                : Name of the detected object.
            hsv_params              (array-like[6, uint8])  : HSV params. See 'hsv_processor' function.
            noise_removal_params    (array-like[9, float32]): Noise removal and morpism params. See 'noise_removal_processor' function.

        Returns:
            bbox_area       (float32)           : Area of the bounding box region in the image.
            bbox_points     (A[4][2, uint32])   : An array of 4 2D points which indicate the bounding box corners.
            points_in_rects (A[N][2, uint32])   : An array of 2D points.
            detection       (bool)              : Returns True when an object is classified.
        """

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
            return self.bbox_points, self.bbox_area, self.points_in_rects, self.detection

        except Exception:
            pass