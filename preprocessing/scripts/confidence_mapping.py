import numpy as np
import cv2

import rospy


class ConfidenceMapping():
    def create_mask(self, data_to_mask, threshold_value):
        """
        Create a masked version of a confidence map
        where every value above a threshold is 1 and every value below is 0.

        Args:
            data_to_mask: the raw confidence map data
            threshold_value: the value to set the threshold. Should be between 0 and 254

        Returns:
            confidence_map_masked: masked confidence data where every value above threshold is 1 and every value below threshold is 0.
            masked_as_cv_image: The masked map as a cv_image
        """
        data_to_mask = self.remove_nans(data_to_mask)
        confidence_map_masked = np.where(data_to_mask <= threshold_value, 0, 1)
        masked_as_cv_image = np.where(data_to_mask <= threshold_value, np.float32(0) , np.float32(100))

        return confidence_map_masked, masked_as_cv_image

    def remove_nans(self, remove_from_here):
        """Remove Nans"""
        nans_removed = np.nan_to_num(remove_from_here)
        return nans_removed

    def add_mask_to_pointcloud(self, confidence_mask, pointcloud_data):
        """
        Adds a confidence mask to the pointcloud data.

        Args:
            confidence_mask: the confidence mask to apply.

        Returns:
            confident_pointcloud: Pointcloud, only with points that have confident data.
        """
        confident_pointcloud = confidence_mask*pointcloud_data
        return confident_pointcloud

    def add_mask_to_rgb(self, confidence_mask, rgb_data):
        """
        Adds a confidence mask to the rgb data.

        Args:
            confidence_mask: the confidence mask to apply.

        Returns:
            confident_rgb: RGB image, only with points that have confident data.
        """
        pass

    def add_mask_to_depth_data(self, confidence_mask, depth_data):
        """
        Adds a confidence mask to the depth data.

        Args:
            confidence_mask: the confidence mask to apply.

        Returns:
            confident_depth: Depth data, only with points that have confident data.
        """

        depth_data = self.remove_nans(depth_data)
        confident_depth = np.multiply(confidence_mask, depth_data)
        return confident_depth


