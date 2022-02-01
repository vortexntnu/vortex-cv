import numpy
import rospy


class ConfidenceMasking():
    def add_mask(self, data_to_mask, threshold_value):
        """
        Returns a masked version of a confidence map,
        where every value above a threshold is 1 and every value below is 0.

        Args:
            data_to_mask: the raw confidence map data
            threshold_value: the value to set the threshold. Should be between 0 and 254

        Returns:
            confidence_map_masked: masked confidence data where every value above threshold is 1 and every value below threshold is 0.
        """
        confidence_map_masked = numpy.where(data_to_mask < threshold_value, 0, 1)
        # confidence_map_masked = numpy.unique(confidence_map_masked)
        return confidence_map_masked