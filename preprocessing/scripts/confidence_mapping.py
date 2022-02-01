import numpy as np
import rospy


class ConfidenceMapping():
    def add_mask(self, data_to_mask, threshold_value, return_map = True):
        """
        Create a masked version of a confidence map
        where every value above a threshold is 1 and every value below is 0.

        Args:
            data_to_mask: the raw confidence map data
            threshold_value: the value to set the threshold. Should be between 0 and 254
            return_map: bool true if you want to return the masked map

        Returns:
            confidence_map_masked: masked confidence data where every value above threshold is 1 and every value below threshold is 0.
        """
        confidence_map_masked = np.where(data_to_mask < threshold_value, 0, 1)
        # confidence_map_masked = np.unique(confidence_map_masked)

        # If return_map is true return the masked map
        if return_map:
            return confidence_map_masked
