#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from gate_detection.cfg import GateDetectionConfig

def callback(config, level):
    rospy.loginfo("""Reconfigure Request: {canny_threshold1}, {canny_threshold2},\ 
          {canny_aperture_size}""".format(**config))
    return config

if __name__ == "__main__":
    rospy.init_node("gate_detection_cfg", anonymous = False)

    srv = Server(GateDetectionConfig, callback)
    rospy.spin()