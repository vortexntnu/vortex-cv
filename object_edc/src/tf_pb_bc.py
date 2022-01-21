#!/usr/bin/env python

import numpy as np
import rospy
import tf2_ros
from tf.transformations import euler_from_quaternion

def tf_pb_bc():
    ros_rate = 2.

    #TF listeners
    tfBuffer = tf2_ros.Buffer()# Add a tf buffer length? tf2_ros.Buffer(rospy.Duration(1200.0))
    listener = tf2_ros.TransformListener(tfBuffer)

    #Constant vector and rotation between body and camera
    body = 'auv/base_link'
    cam_front = 'auv/camerafront_link'

    #listener.waitForTransform(body, cam_front, rospy.Time(), rospy.Duration(10.0))

    while tfBuffer.can_transform(body, cam_front, rospy.Time()) == 0:
        try:
            rospy.loginfo(str(body) +' and ' + str(cam_front) + ' not found help')
            rospy.sleep(ros_rate)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.sleep(ros_rate)
            continue
   
    tf_lookup_bc = tfBuffer.lookup_transform(body, cam_front, rospy.Time.now(), rospy.Duration(1.0))
    rospy.loginfo("Transformation between body and camera: %s ", tf_lookup_bc)
    pb_bc = np.array([tf_lookup_bc.transform.translation.x, tf_lookup_bc.transform.translation.y, tf_lookup_bc.transform.translation.z])
    rospy.loginfo(tf_lookup_bc.transform.rotation)
    explicit_quat = [tf_lookup_bc.transform.rotation.x, tf_lookup_bc.transform.rotation.y, tf_lookup_bc.transform.rotation.z, tf_lookup_bc.transform.rotation.w ]
    euler_bc = np.array(euler_from_quaternion(explicit_quat))
    

    return pb_bc, euler_bc
