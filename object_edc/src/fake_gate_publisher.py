#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped


class PublishNode():
    def __init__(self):
        rospy.init_node("fake_publisher") #,  log_level=rospy.DEBUG
        self.sleep_rate = 1.
        
        self.pub = rospy.Publisher('/object_detection/object_pose/gate', PoseStamped, queue_size=1)
        #self.pub_true = rospy.Publisher('/object_detection/object_pose/gate_truth', PoseStamped, queue_size=1)
        rospy.sleep(self.sleep_rate)

    
    def callback(self):
        while not rospy.is_shutdown():
            n = np.random.normal(0, 0.1**2, 3)
            n_2 = np.random.normal(0, 0.2**2, 3)

            gate = PoseStamped()
            gate.header.frame_id = "1"
            gate.pose.position.x = 2.30953173828
            gate.pose.position.y = -0.274446075439
            gate.pose.position.z = 1.12427368164
            gate.pose.orientation.x = 0.0168736690947
            gate.pose.orientation.y = 0.0740790786578
            gate.pose.orientation.z = 0.0434789330927
            gate.pose.orientation.w = 0.99616120054
            rospy.loginfo(gate)
            self.pub.publish(gate)

            rospy.sleep(self.sleep_rate) #Publish rate

if __name__ == '__main__':
    try:
        ekf_vision = PublishNode()
        ekf_vision.callback()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
