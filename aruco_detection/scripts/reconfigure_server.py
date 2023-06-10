#!/usr/bin/env python

import rospy
import rospkg
import numpy as np
import yaml
from dynamic_reconfigure.server import Server

from aruco_detection.cfg import imgFilterConfig


class ReconfigureImageFilter:

    def __init__(self):
        rospy.init_node("reconfigure_server")
        self.srv = Server(imgFilterConfig, self.reconfigure_cb)

    def reconfigure_cb(self, config, level):
        return config


if __name__ == "__main__":
    try:
        reconf = ReconfigureImageFilter()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass