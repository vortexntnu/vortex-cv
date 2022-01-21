#!/usr/bin/env python

##EKF imports
#from logging import exception
from re import X
from ekf_filtering.ekf.ekf_python2.gaussparams_py2 import MultiVarGaussian
from ekf_filtering.ekf.ekf_python2.dynamicmodels_py2 import landmark_gate
from ekf_filtering.ekf.ekf_python2.measurementmodels_py2 import NED_range_bearing
from ekf_filtering.ekf.ekf_python2.ekf_py2 import EKF
#from importlib.metadata import distribution

#ekf_node_imports
from src.transform_to_world import transform_world_to_gate
from src.tf_pb_bc import tf_pb_bc

#Math imports
import numpy as np
import math
from scipy.spatial.transform import Rotation

#ROS imports
import rospy
from vortex_msgs.msg import ObjectPosition
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler

class EKFNode:
    ## gate state: x = [p_wg, theta_wg]
    

    def __init__(self):
        ##################
        ####EKF stuff####
        ##################

        # Geometric parameters
        self.rad2deg = 180 / np.pi
        self.deg2rad = np.pi / 180

        self.gate_prior = [27 , 0, 0] # z, roll, pitch of gate

        self.pb_bc = [0.3175, 0, - 0.10]
        self.euler_bc = [self.deg2rad*0.5, self.deg2rad*17.3, 0]
        
        # Tuning parameters
        self.sigma_a = np.array([0.5, 0.5, 0.5, 0.5])
        self.sigma_z = np.array([0.05, 0.05, 0.05, 0.05])

        # Making gate model object
        self.gate_model = landmark_gate(self.sigma_a)

        #Gauss prev values

        self.x_hat0 = np.array([0, 0, 0, 0]) 
        self.P_hat0 = np.diag(self.sigma_z)
        self.prev_gauss = MultiVarGaussian(self.x_hat0, self.P_hat0)

        ################
        ###ROS stuff####
        ################

        # ROS node init
        rospy.init_node('ekf_vision')

        # Subscriber to gate pose and orientation #ToDo update the message type received from cv
        self.object_pose_sub = rospy.Subscriber('/object_detection/object_pose/gate', PoseStamped, self.obj_pose_callback, queue_size=1)
        self.obj_pose = 0
        self.obj_pose_prev = PoseStamped()

        # Subscriber to Odometry 
        self.odometry_sub = rospy.Subscriber('/odometry/filtered', Odometry, self.odometry_callback, queue_size=1)

        # Publisher #ToDo update the message type that autonomous wants (Vortex-AUV/vortex_msgs/msg/ObjectPosition.msg)
        self.gate_pose_pub = rospy.Publisher('/fsm/object_positions_in', ObjectPosition, queue_size=1)

        #Constant vector and rotation between body and camera
        #self.pb_bc, self.euler_bc = tf_pb_bc()


    def get_Ts(self):
        Ts = rospy.get_time() - self.last_time
        return Ts
    
    def ekf_function(self, pw_wc, Rot_wb, z):

        measurement_model = NED_range_bearing(self.sigma_z, pw_wc, Rot_wb)

        Ts = self.get_Ts()

        my_ekf = EKF(self.gate_model, measurement_model)

        gauss_x_pred, gauss_z_pred, gauss_est = my_ekf.step_with_info(self.prev_gauss, z, Ts)
        
        self.last_time = rospy.get_time()
        self.prev_gauss = gauss_est # instantiate last estimate and time

        return gauss_x_pred, gauss_z_pred, gauss_est

    def est_to_pose(self, x_hat):
        x = x_hat[0]
        y = x_hat[1]
        z = self.gate_prior[0]
        pos = [x, y, z]

        euler_angs = [self.gate_prior[1], self.gate_prior[2], x_hat[3]]

        return pos, euler_angs

    def obj_pose_callback(self, msg):
        self.obj_pose = PoseStamped()
        self.obj_pose = msg

    def odometry_callback(self, msg):
        odom_pose = Odometry()
        odom_pose = msg
        
        if self.obj_pose == 0 or self.obj_pose.pose.position.x == self.obj_pose_prev.pose.position.x:
            rospy.loginfo("No new object pose")
            rospy.sleep(2.)
            pass
        else:   
            rospy.loginfo("New obj pose")
            self.obj_pose_prev = self.obj_pose 
            pw_wc, Rot_wc, z = transform_world_to_gate(odom_pose, self.obj_pose, self.pb_bc, self.euler_bc)



            #Do ekf here
            gauss_x_pred, gauss_z_pred, gauss_est = self.ekf_function(pw_wc, Rot_wc, z)
            x_hat = gauss_est.mean


            #EKF data pub
            position, pose = self.est_to_pose(x_hat)
            pose_quaterion = quaternion_from_euler(pose)


            p = ObjectPosition() 
            #p.pose.header[]
            p.objectID = self.obj_pose.header.frame_id
            p.pose.position.x = position[0]
            p.pose.position.y = position[1]
            p.pose.position.z = position[2]
            p.pose.orientation.x = pose_quaterion[0]
            p.pose.orientation.y = pose_quaterion[1]
            p.pose.orientation.z = pose_quaterion[2]
            p.pose.orientation.w = pose_quaterion[3]
            rospy.loginfo("Published: %s ", p)

            #Publish data
            self.gate_pose_pub.publish(p)
    
if __name__ == '__main__':
    try:
        ekf_vision = EKFNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    