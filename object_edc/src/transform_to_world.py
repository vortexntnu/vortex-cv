#!/usr/bin/env python

import numpy as np
import math
import rospy
from tf.transformations import euler_from_quaternion
from scipy.spatial.transform import Rotation as R

def transform_world_to_gate(odom, obj_pose, pb_bc, euler_bc): #msg=Odometry, obj_pose=ObjectPosition, pb_bc=[x,y,z], euler_bc=[x,y,z]
    
    #Gate
    gate = obj_pose
    pc_cg = np.array([gate.pose.position.x, gate.pose.position.y, gate.pose.position.z])
    gate_explicit_quat = np.array([gate.pose.orientation.x, gate.pose.orientation.y, gate.pose.orientation.z, gate.pose.orientation.w ])

    #Odometry (world to baselink)
    pw_wb = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z])
    odom_explicit_quat = [odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w]
    drone_orientation_euler = np.array(euler_from_quaternion(odom_explicit_quat))

    #Getting rotation matrices
    Rot_bc = euler2Rot_wb(euler_bc)
    Rot_wb = euler2Rot_wb(drone_orientation_euler)

    # Rot_bc = R.from_euler(euler_bc)
    # Rot_wb = R.from_euler(drone_orientation_euler)
    Rot_wc = np.matmul(Rot_wb, Rot_bc) #Rot_wb @ Rot_bc


    #Camera world vector
    pw_wc = pw_wb + np.dot(Rot_wb, pb_bc) #Rot_wb @ self.pb_bc

    #Angle
    gamma_gc = get_measurement_angle(gate_explicit_quat)
    gamma_wc = gamma_gc + np.pi + drone_orientation_euler[2]

    z = [np.dot(Rot_wc, pc_cg), gamma_wc] #Rot_wc @ pc_cg

    #rospy.loginfo("pw_wc")
    #rospy.loginfo(pw_wc)
    #rospy.loginfo("gamma_gc")
    #rospy.loginfo(gamma_gc)
    #rospy.loginfo("gamma_wc")
    #rospy.loginfo(gamma_wc)

    return pw_wc, Rot_wc, z

def get_measurement_angle(quaternion):
    euler_p = euler_from_quaternion(quaternion)
    euler_m = euler_from_quaternion(-1*quaternion)
    return min(euler_p[2], euler_m[2])

def euler2Rot_wb(euler):
    phi = euler[0]
    theta = euler[1]
    psi = euler[2]

    Rot_xphi = np.array([[1, 0, 0], [0, math.cos(phi), -math.sin(phi)], [0, math.sin(phi), math.cos(phi)]])
    Rot_ytheta = np.array([[math.cos(theta), 0, math.sin(theta)], [0, 1, 0], [-math.sin(theta), 0, math.cos(theta)]])
    Rot_zpsi = np.array([[math.cos(psi), -math.sin(psi), 0], [math.sin(psi), math.cos(psi), 0], [0, 0, 1]])

    Rot_wb = np.linalg.multi_dot([Rot_zpsi ,Rot_ytheta ,Rot_xphi]) #Rot_wb = Rot_zpsi @ Rot_ytheta @ Rot_xphi
    return Rot_wb