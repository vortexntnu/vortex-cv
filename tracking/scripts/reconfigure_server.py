#!/usr/bin/python3

import rospy
import rospkg
import numpy as np
import yaml
from dynamic_reconfigure.server import Server

from tracking.cfg import TrackingConfig


class ReconfigureTracker:
    def __init__(self):
        rospy.init_node("reconfigure_server")
        self.srv = Server(TrackingConfig, self.reconfigure_cb)

    def reconfigure_cb(self, config, level):
        return config


if __name__ == "__main__":
    try:
        reconf = ReconfigureTracker()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
