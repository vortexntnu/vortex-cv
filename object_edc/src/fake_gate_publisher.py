#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

class PublishNode():
    def __init__(self):
        rospy.init_node("fake_publisher") #,  log_level=rospy.DEBUG
        self.sleep_rate = 10.
        
        self.pub = rospy.Publisher('/object_detection/object_pose/gate', PoseStamped)
        rospy.sleep(self.sleep_rate)

    
    def callback(self):
        gate = PoseStamped()
        gate.header.frame_id = "1"
        gate.pose.position.x = 5
        gate.pose.position.y = 1
        gate.pose.position.z = 1
        gate.pose.orientation.x = 0 
        gate.pose.orientation.y = 0
        gate.pose.orientation.z = 0.3826834
        gate.pose.orientation.w = 0.9238795
        
        while not rospy.is_shutdown():
            self.pub.publish(gate)
            rospy.loginfo(gate)
            rospy.sleep(self.sleep_rate)

if __name__ == '__main__':
    try:
        ekf_vision = PublishNode()
        ekf_vision.callback()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
