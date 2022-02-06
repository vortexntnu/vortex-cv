#!/usr/bin/env python

import numpy as np
import rospy
import tf2_ros
from tf.transformations import euler_from_quaternion

def tf_pb_bc(parent_frame, child_frame):
    ros_rate = 2.
    
    #TF listeners
    tfBuffer = tf2_ros.Buffer()# Add a tf buffer length? tf2_ros.Buffer(rospy.Duration(1200.0))
    listener = tf2_ros.TransformListener(tfBuffer)
    while not rospy.is_shutdown():    
        while tfBuffer.can_transform(parent_frame, child_frame, rospy.Time()) == 0:
            try:
                rospy.loginfo("No transform between "+str(parent_frame) +' and ' + str(child_frame))
                rospy.sleep(ros_rate)

            except: #, tf2_ros.ExtrapolationException  (tf2_ros.LookupException, tf2_ros.ConnectivityException)
                rospy.sleep(ros_rate)
                continue
        tf_lookup_bc = tfBuffer.lookup_transform(parent_frame, child_frame, rospy.Time())
        break
    
    return tf_lookup_bc

