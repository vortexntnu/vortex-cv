#!/usr/bin/env python

# import debugpy
# print("Waiting for VSCode debugger...")
# debugpy.listen(5678)
# debugpy.wait_for_client()

##EKF imports
#from logging import exception
from re import X

from scipy import matmul
#from statistics import geometric_mean
from ekf_python2.gaussparams_py2 import MultiVarGaussian
from ekf_python2.dynamicmodels_py2 import landmark_gate
from ekf_python2.measurementmodels_py2 import NED_range_bearing
from ekf_python2.ekf_py2 import EKF
#from importlib.metadata import distribution

#Math imports
import numpy as np
import math
from scipy.spatial.transform import Rotation
from scipy.spatial.transform import Rotation as R

#ekf_node_imports
#from transform_to_world import transform_world_to_gate
from tf_pb_bc import tf_pb_bc

#ROS imports
import rospy
from vortex_msgs.msg import ObjectPosition
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
import tf.transformations as tft
import tf2_ros
import tf2_py 

class EKFNode:
    ## gate state: x = [p_wg, theta_wg]
    

    def __init__(self):
        
        ##################
        ####EKF stuff####
        ##################

        # Geometric parameters
        self.rad2deg = 180 / np.pi
        self.deg2rad = np.pi / 180
        self.gate_prior = [0, 0] # z, roll, pitch of gate

        # Tuning parameters
        self.sigma_a = np.array([0.05, 0.05, 0.05, 0.05])
        self.sigma_z = np.array([0.5, 0.5, 0.5, 0.5])

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
        self.last_time = rospy.get_time()

        now = rospy.get_rostime()
        rospy.loginfo("Current time %i %i", now.secs, now.nsecs)

        self.__tfBuffer = tf2_ros.Buffer()# TODO: Add a tf buffer length? tf2_ros.Buffer(rospy.Duration(1200.0))
        self.__listener = tf2_ros.TransformListener(self.__tfBuffer)
        self.__tfBroadcaster = tf2_ros.TransformBroadcaster()

        #Transform names
        self.odom = 'mocap' #'mcap' for cyb pool, TODO 'odom'? for regular
        self.body = 'Body_1' 
        self.cam = 'auv/camerafront_link'
        self.gate_truth = 'gate_truth'
        
        self.pb_bc = 0
        self.tf_lookup_bc = tf_pb_bc(self.odom, self.cam)
        rospy.loginfo("Transformation between odom and camera published")

        # Subscriber to gate pose and orientation #ToDo update the message type received from cv
        self.object_pose_sub = rospy.Subscriber('/object_detection/object_pose/gate', PoseStamped, self.obj_pose_callback, queue_size=1)
        self.obj_pose_prev = PoseStamped()
      
        # Publisher to autonomous
        self.gate_pose_pub = rospy.Publisher('/fsm/object_positions_in', ObjectPosition, queue_size=1)


    def get_Ts(self):
        Ts = rospy.get_time() - self.last_time
        return Ts
    
    def ekf_function(self, pw_wc, Rot_wc, z):

        measurement_model = NED_range_bearing(self.sigma_z, pw_wc, Rot_wc)

        Ts = self.get_Ts()

        my_ekf = EKF(self.gate_model, measurement_model)

        gauss_x_pred, gauss_z_pred, gauss_est = my_ekf.step_with_info(self.prev_gauss, z, Ts)
        
        self.last_time = rospy.get_time()
        self.prev_gauss = gauss_est # instantiate last estimate and time

        return gauss_x_pred, gauss_z_pred, gauss_est

    def est_to_pose(self, x_hat):
        x = x_hat[0]
        y = x_hat[1]
        z = x_hat[2]
        pos = [x, y, z]

        euler_angs = [self.gate_prior[0], self.gate_prior[1], x_hat[3]]
        return pos, euler_angs

    def transformbroadcast(self, parent_frame, p):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = parent_frame
        t.child_frame_id = "object_"+str(p.objectID)
        t.transform.translation.x = p.objectPose.pose.position.x
        t.transform.translation.y = p.objectPose.pose.position.y
        t.transform.translation.z = p.objectPose.pose.position.z
        t.transform.rotation.x = p.objectPose.pose.orientation.x
        t.transform.rotation.y = p.objectPose.pose.orientation.y
        t.transform.rotation.z = p.objectPose.pose.orientation.z
        t.transform.rotation.w = p.objectPose.pose.orientation.w
        self.__tfBroadcaster.sendTransform(t)

        

    def publish_gate(self, object_name, ekf_position, ekf_pose_quaterion):
        p = ObjectPosition() 
        #p.pose.header[]
        p.objectID = object_name
        p.objectPose.pose.position.x = ekf_position[0]
        p.objectPose.pose.position.y = ekf_position[1]
        p.objectPose.pose.position.z = ekf_position[2]
        p.objectPose.pose.orientation.x = ekf_pose_quaterion[0]
        p.objectPose.pose.orientation.y = ekf_pose_quaterion[1]
        p.objectPose.pose.orientation.z = ekf_pose_quaterion[2]
        p.objectPose.pose.orientation.w = ekf_pose_quaterion[3]
        
        self.gate_pose_pub.publish(p)
        rospy.loginfo("Object published: %s", object_name)
        self.transformbroadcast(self.odom, p)

    def obj_pose_callback(self, msg):
        rospy.loginfo("Object data recieved for: %s", msg.header.frame_id)
        #Gate in world frame for cyb pool
        obj_pose_position_w = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        #Go directly from quaternion to matrix
        obj_pose_pose_w = np.array([msg.pose.orientation.x,
                                    msg.pose.orientation.y,
                                    msg.pose.orientation.z,
                                    msg.pose.orientation.w])
 

        tf_lookup_wc = self.__tfBuffer.lookup_transform(self.odom, self.cam, rospy.Time(), rospy.Duration(5.0))

        # Assumption: this is the matrix that transforms a vector from world to camera (parent to child)
        # New working assumption: this is actually from child to parent (camera to world)
        # The new working assumption is the current best estimate.. kill me
        Rot_wc = tft.quaternion_matrix([tf_lookup_wc.transform.rotation.x, 
                                        tf_lookup_wc.transform.rotation.y,
                                        tf_lookup_wc.transform.rotation.z,
                                        tf_lookup_wc.transform.rotation.w])

        pw_wc = np.array([tf_lookup_wc.transform.translation.x,
                          tf_lookup_wc.transform.translation.y,
                          tf_lookup_wc.transform.translation.z])

        #Rot_wc = tft.euler_matrix(tf_lookup_wc_euler[0], 
        #                          tf_lookup_wc_euler[1],
        #                          tf_lookup_wc_euler[2], axes = "sxyz")

        Rot_wc = Rot_wc[0:3, 0:3]
        
        
        #TODO fix ekf value inputs
        # 1. Generate measurement. We use only tf because fuck doing this ourselves : 
        tf_lookup_cg = self.__tfBuffer.lookup_transform(self.cam, self.gate_truth, rospy.Time(), rospy.Duration(5.0))

        pc_cg =  np.array([tf_lookup_cg.transform.translation.x,
                          tf_lookup_cg.transform.translation.y,
                          tf_lookup_cg.transform.translation.z]) 

        #pc_cg = np.matmul(np.transpose(Rot_wc), (obj_pose_position_w - pw_wc))
        gamma_wc = 1
        z = pc_cg # This might be wrong, 
        z = np.append(z, gamma_wc) 
        
 
        #Do ekf here
        gauss_x_pred, gauss_z_pred, gauss_est = self.ekf_function(pw_wc, Rot_wc, z)
        x_hat = gauss_est.mean
        #EKF data pub
        ekf_position, ekf_pose = self.est_to_pose(x_hat)
        ekf_pose_quaterion = tft.quaternion_from_euler(ekf_pose[0], ekf_pose[1], ekf_pose[2])

        #Publish data
        self.publish_gate(msg.header.frame_id ,ekf_position, ekf_pose_quaterion)
        self.obj_pose_prev = msg
       



if __name__ == '__main__':
    while not rospy.is_shutdown():     
        try:
            ekf_vision = EKFNode()
            rospy.spin()
        except rospy.ROSInterruptException:
            pass
    