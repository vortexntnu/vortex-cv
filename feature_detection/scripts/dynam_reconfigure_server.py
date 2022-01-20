#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from feature_detection.cfg import FeatureDetectionConfig

def callback(config, level):
    # rospy.loginfo("""Reconfigure Request: {canny_threshold1}, {canny_threshold2},\ 
    #       {canny_aperture_size}""".format(**config))
    return config

if __name__ == "__main__":
    rospy.init_node("feature_detection_cfg", anonymous = False)

    srv = Server(FeatureDetectionConfig, callback)
    rospy.spin()