
import numpy as np
import rospy
import ros_numpy
from sensor_msgs.msg import PointCloud2

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
        """
        ***Remove Nans***\n
        Removes all nans and infs from a numpy.ndarray.

        Args:
            remove_from_here: The nympy.ndarray to remove from

        Returns:
            nans_removed: numpy.ndarray with nans replaced by zeros and infs replaced by finite large numbers.
        """
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
        numpified_pointcloud = ros_numpy.numpify(pointcloud_data)
        # numpified_pointcloud = np.nan_to_num(numpified_pointcloud)
        # rospy.loginfo(np.shape(numpified_pointcloud))
        # rospy.loginfo("Numpified type is: %s and value is: %s ", type(numpified_pointcloud[2][1]), numpified_pointcloud[2][1])


        # confident_numpified_pointcloud = np.where(confidence_mask == 1, numpified_pointcloud, confidence_mask)
        # rospy.loginfo("Numpified type is: %s and value is: %s ", type(confident_numpified_pointcloud[2][1]), confident_numpified_pointcloud[2][1])
        # confident_pointcloud = ros_numpy.msgify(PointCloud2, confident_numpified_pointcloud)
        # return confident_pointcloud

    def add_mask_to_cv_image(self, confidence_mask, cv_image):
        """
        Adds a confidence map to a cv image. Elementwise multiplication between confidence mask and cv image. Must be same size and shape

        Args:
            confidence_mask: the confidence mask to apply.
            cv_image: the cv image to apply the mask to.

        Returns:
            confident_cv_image: a confident representation of a cv image.
        """

        cv_image = self.remove_nans(cv_image)
        confident_cv_image = np.multiply(confidence_mask, cv_image, dtype=np.float32)
        return confident_cv_image


