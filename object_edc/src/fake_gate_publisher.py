#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

class PublishNode():
    def __init__(self):
        rospy.init_node("fake_publisher") #,  log_level=rospy.DEBUG
        self.sleep_rate = 2.
        
        self.pub = rospy.Publisher('/object_detection/object_pose/gate', PoseStamped, queue_size=1)
        self.pub_true = rospy.Publisher('/object_detection/object_pose/gate_truth', PoseStamped, queue_size=1)
        rospy.sleep(self.sleep_rate)

    
    def callback(self):
        while not rospy.is_shutdown():
            n = np.random.normal(0, 0.1**2, 3)
            n_2 = np.random.normal(0, 0.2**2, 3)

            gate = PoseStamped()
            gate.header.frame_id = "1"
            #gate.pose.position.x = np.random.random()
            #gate.pose.position.y = np.random.random()
            #gate.pose.position.z = 1
            #gate.pose.orientation.x = 0 
            #gate.pose.orientation.y = 0
            #gate.pose.orientation.z = 0.3826834
            #gate.pose.orientation.w = 0.9238795
            gate.pose.position.x = 3 + n[0]
            gate.pose.position.y = 1 + n[1]
            gate.pose.position.z = -0.5 + n[2]
            gate.pose.orientation.x = 0
            gate.pose.orientation.y = 0
            gate.pose.orientation.z = 0.3826834
            gate.pose.orientation.w = 0.9238795
            rospy.loginfo(gate)
            self.pub.publish(gate)

            #gate_true = PoseStamped()
            #gate_true.header.frame_id = "2"
            #gate_true.pose.position.x = 5 
            #gate_true.pose.position.y = 2
            #gate_true.pose.position.z = 1
            #gate_true.pose.orientation.x = 0 
            #gate_true.pose.orientation.y = 0
            #gate_true.pose.orientation.z = 0.3826834
            #gate_true.pose.orientation.w = 0.923879
            #self.pub_true.publish(gate_true)

            #rospy.loginfo(gate)
            rospy.sleep(self.sleep_rate)

if __name__ == '__main__':
    try:
        ekf_vision = PublishNode()
        ekf_vision.callback()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
