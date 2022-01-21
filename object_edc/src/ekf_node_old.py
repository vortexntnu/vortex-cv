#!/usr/bin/env python

from logging import exception
from ekf_python2.gaussparams_py2 import MultiVarGaussian
import rospy
import tf2_ros
import numpy as np
import math
from ekf_python2.dynamicmodels_py2 import landmark_gate
from ekf_python2.measurementmodels_py2 import NED_range_bearing

from ekf.ekf_python2.ekf_py2 import EKF

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class EKFNode:
    ## gate state: x = [p_wg, theta_wg]
    

    def __init__(self):
        ##################
        ####EKF stuff####
        ##################

        # Geometric parameters
        self.gate_prior = [27 , 0, 0] # z, roll, pitch of gate
        # Tuning parameters
        self.sigma_a = np.array([0.5, 0.5, 0.5, 0.5])
        self.sigma_z = np.array([0.5, 0.5, 0.5, 0.5])

        # Making gate model object
        self.gate_model = landmark_gate(self.sigma_a)

        #Initiation of EKF

        self.x_hat0 = np.array([0, 0, 0, 0]) 
        self.P_hat0 = np.diag(self.sigma_z)
        self.prev_gauss = MultiVarGaussian(self.x_hat0, self.P_hat0)

        ################
        ###ROS stuff####
        ################

        # ROS node init
        rospy.init_node('ekf_vision')

        # Subscriber to gate pose and orientation #ToDo update the message type received from cv
        self.object_detection_sub = rospy.Subscriber('/object_detection/object_pose/gate', PoseStamped) #, self.gate_pose_callback, queue_size=1

        # Subscriber to Odometry 
        self.odometry_sub = rospy.Subscriber('/odometry/filtered', Odometry, self.odometry_callback, queue_size=1)

        # Publisher #ToDo update the message type that autonomous wants (Vortex-AUV/vortex_msgs/msg/ObjectPosition.msg)
        self.gate_pose_pub = rospy.Publisher('/fsm/object_positions_in', PoseStamped, queue_size=1)

        #TF listeners
        self.tfBuffer = tf2_ros.Buffer()# Add a tf buffer length? tf2_ros.Buffer(rospy.Duration(1200.0))
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        #TF frames beeing use
        #self.world_ned = 'world_ned' #ToDo Check if 'world_ned' frame is beeing published on /tf
        self.body = 'auv/base_link'
        self.cam_front = 'auv/camerafront_link'

        #Constant vector and rotation between body and camera
        #While-loop that checks if a frame body and camera has been published
        tf_check = 0
        while tf_check == 0:
            if self.tf.frameExists(self.body) and self.tf.frameExists(self.camerafront_link):
                tf_check = 1
                rospy.sleep(2.)
            else:
                rospy.loginfo(str(self.body) +' and ' + str(self.camerafront_link) + 'not found')
                continue
        
        tf_lookup_bc = self.tfBuffer.lookup_transform(self.body, self.camerafront_link, rospy.Time.now(), rospy.Duration(1.0))
        rospy.loginfo("Transformation between body and camera: %s ", tf_lookup_bc)
        self.pb_bc = np.array(tf_lookup_bc.transform.translation) #Vector3(x,y,z)
        self.euler_bc = np.array(euler_from_quaternion(tf_lookup_bc.transform.orientation))
        
    def get_Ts(self):
        Ts = rospy.get_time() - self.last_time
        return Ts
    
    def euler2Rot_wb(self, euler):
        phi = euler[0]
        theta = euler[1]
        psi = euler[2]

        Rot_xphi = np.array([[1, 0, 0], [0, math.cos(phi), -math.sin(phi)], [0, math.sin(phi), math.cos(phi)]])
        Rot_ytheta = np.array([[math.cos(theta), 0, math.sin(theta)], [0, 1, 0], [-math.sin(theta), 0, math.cos(theta)]])
        Rot_zpsi = np.array([[math.cos(psi), -math.sin(psi), 0], [math.sin(psi), math.cos(psi), 0], [0, 0, 1]])

        Rot_wb = np.linalg.multi_dot([Rot_zpsi ,Rot_ytheta ,Rot_xphi]) #Rot_wb = Rot_zpsi @ Rot_ytheta @ Rot_xphi
        
        return Rot_wb


    #ta in fra "/object_detection/object_pose/gate"
    def get_measurement_angle(quaternion):

        euler_p = euler_from_quaternion(quaternion)
        euler_m = euler_from_quaternion(-quaternion)

        return min(euler_p[2], euler_m[2])
    


    def ekf_function(self, pw_wc, Rot_wb, z):

        measurement_model = NED_range_bearing(self.sigma_z, pw_wc, Rot_wb)


        Ts = self.get_Ts()

        my_ekf = EKF(self.gate_model, measurement_model)

        gauss_x_pred, gauss_z_pred, gauss_est = my_ekf.step_with_info(self.prev_gauss, z, Ts)
        
        self.last_time = rospy.get_time()
        self.prev_gauss = gauss_est # instantiate last estimate and time

        return gauss_x_pred, gauss_z_pred, gauss_est

    def transform_world_to_gate(self, msg):
        
        #Vector 
        gate = self.obj_pose #PoseStamped

        pc_cg = np.array(gate.pose.position)

        gate_quaternion = gate.pose.orientation
        gate_orientation_euler = euler_from_quaternion(gate_quaternion) # x,y,z,w
        
        #Odometry (world to baselink)
        pw_wb = np.array(msg.pose.pose.position)
        drone_orientation_euler = np.array(euler_from_quaternion(msg.pose.pose.orientation))

        Rot_bc = self.euler2Rot_wb(self.euler_bc)
        Rot_wb = self.euler2Rot_wb(drone_orientation_euler)

        Rot_wc = np.dot([Rot_wb, Rot_bc]) #Rot_wb @ Rot_bc

        #Or do Transform between two frames with
        while not rospy.is_shutdown():
            try:
                tf_lookup_bc = self.tfBuffer.lookup_transform(self.auv, self.camerafront_link, rospy.Time.now(), rospy.Duration(1.0))
                pw_bc = np.array(tf_lookup_bc.transform.translation) #Vector3(x,y,z)
                pw_wc = pw_wb + np.dot(Rot_wb, self.pb_bc) #Rot_wb @ self.pb_bc
                gamma_gc = self.get_measurement_angle(gate_quaternion)
            except Exception:
                print("aksjnfdakjfsnk")
                # Still #ToDo

        pw_wc = pw_wb + np.dot(Rot_wb, self.pb_bc) #Rot_wb @ self.pb_bc
        gamma_wc = gamma_gc + np.pi + drone_orientation_euler[2]
    

        z = [np.dot(Rot_wc, pc_cg), gamma_wc] #Rot_wc @ pc_cg
        return pw_wc, Rot_wc, z

    def obj_pose_callback(self, msg):
        self.obj_pose = msg

    def odometry_callback(self, msg):

        pw_wc, Rot_wc, z = self.transform_world_to_gate(msg)

        #Do ekf here
        self.ekf_function(pw_wc, Rot_wc, z)
        
        #EKF data pub
        #position, pose = est_to_pose(x_hat)
        #pose_quaterion = quaternion_from_euler(pose)

        while not rospy.is_shutdown():
            try:
                p = PoseStamped()
                #p.pose.header[]
                p.pose.position = []
                p.pose.orientation = []
                '''
                #p.header.seq =
                #p.header.stamp = 
                #p.header.frame_id =
                p.pose.position.x = position[0]
                p.pose.position.y = position[1]
                p.pose.position.z = position[2]
                p.pose.orientation.x = pose_quaterion[0]
                p.pose.orientation.y = pose_quaterion[1]
                p.pose.orientation.z = pose_quaterion[2]
                p.pose.orientation.w = pose_quaterion[3]
                '''
                rospy.loginfo("Published: %s ", p)

                #Publish data
                self.gate_pose_pub.publish(p)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                self.rate.sleep()
                continue 

if __name__ == '__main__':
    try:
        ekf_vision = EKFNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    
    

    '''
    #Transform between two frames
    while not rospy.is_shutdown():
        try:
            tf_lookup_wb = self.tfBuffer.lookup_transform(self.world_ned, self.camerafront_link, rospy.Time.now(), rospy.Duration(1.0))
            #If lookup_transform fails, the ekf might fail
            pw_wc = np.array(tf_lookup_wb.transform.translation) #Vector3(x,y,z)
            pc_wg = gate_points - pw_wc 
            Rot_wc_quaterions = tf_lookup_wb.transform.rotation #Rotation in quaterions
            Rot_wc_euler = np.array(euler_from_quaternion(Rot_wc_quaterions)) #Rotation convertion euler from quaterions 
    '''   
