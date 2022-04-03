#!/usr/bin/env python

#import debugpy
#print("Waiting for VSCode debugger...")
#debugpy.listen(5678)
#debugpy.wait_for_client()

##EKF imports
#from logging import exception
from re import X

from ekf_python2.gaussparams_py2 import MultiVarGaussian
from ekf_python2.dynamicmodels_py2 import landmark_gate
from ekf_python2.measurementmodels_py2 import measurement_linear_landmark
from ekf_python2.ekf_py2 import EKF

#Math imports
import numpy as np

#ROS imports
import rospy
from std_msgs.msg import String
from vortex_msgs.msg import ObjectPosition
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf.transformations as tft
import tf2_ros

# GMF imports
from scipy.stats import chi2

class GMMFNode:
    

    def __init__(self):
        ########################################
        ####Things you can change yourself####
        ########################################

        #Name of the node
        node_name = "gmf"

        #Frame names, e.g. "odom" and "cam"

        ## TODO: fix after talk with Finn and Tarek
        self.parent_frame = 'odom' 
        self.child_frame = 'zed2_left_camera_frame'
        self.object_frame = ""

        self.current_object = ""
        
        #Subscribe topic
        object_topic_subscribe = "/pointcloud_processing/object_pose/spy"
        mission_topic_subscribe = "/fsm/state"

        ##################
        #### GMF stuff####
        ##################
        ndim = 6
        gate_percentile = 0.99

        self.gate_size_sq = chi2.ppf(gate_percentile, ndim)


        ##################
        ####EKF stuff#####
        ##################

        # Geometric parameters
        #self.gate_prior = [0, 0] # z, roll, pitch of gate

        # Tuning parameters
        self.sigma_a = 3/5*np.array([0.05, 0.05, 0.05, 0.05, 0.05, 0.05])
        self.sigma_z = 2*np.array([0.5, 0.5, 0.5, 0.5, 0.5, 0.5])

        # Making gate model object
        self.landmark_model = landmark_gate(self.sigma_a)
        self.sensor_model = measurement_linear_landmark(self.sigma_z)
        self.my_ekf = EKF(self.landmark_model, self.sensor_model)

        #Gauss prev values
        self.x_hat0 = np.array([0, 0, 0, 0, 0, 0])
        self.P_hat0 = np.diag(3*self.sigma_z)
        self.prev_gauss = MultiVarGaussian(self.x_hat0, self.P_hat0)

        ################
        ###ROS stuff####
        ################

        # ROS node init
        rospy.init_node(node_name)
        self.last_time = rospy.get_time()

        now = rospy.get_rostime()
        rospy.loginfo("Current time %i %i", now.secs, now.nsecs)

        # Subscribe to mission topic
        self.mission_topic = self.current_object + "_execute"
        self.mission_topic_sub = rospy.Subscriber(mission_topic_subscribe, String, self.update_mission)
        # Subscriber to gate pose and orientation 
        self.object_pose_sub = rospy.Subscriber(object_topic_subscribe, ObjectPosition, self.obj_pose_callback, queue_size=1)
      
        # Publisher to autonomous
        self.gate_pose_pub = rospy.Publisher('/fsm/object_positions_in', ObjectPosition, queue_size=1)

        #TF stuff
        self.__tfBuffer = tf2_ros.Buffer() # TODO Add a tf buffer length? tf2_ros.Buffer(rospy.Duration(1200.0)) (Kristian)
        self.__listener = tf2_ros.TransformListener(self.__tfBuffer)
        self.__tfBroadcaster = tf2_ros.TransformBroadcaster()

        #The init will only continue if a transform between parent frame and child frame can be found
        while self.__tfBuffer.can_transform(self.parent_frame, self.child_frame, rospy.Time()) == 0:
            try:
                rospy.loginfo("No transform between "+str(self.parent_frame) +' and ' + str(self.child_frame))
                rospy.sleep(2)
            except: #, tf2_ros.ExtrapolationException  (tf2_ros.LookupException, tf2_ros.ConnectivityException)
                rospy.sleep(2)
                continue
        
        rospy.loginfo("Transform between "+str(self.parent_frame) +' and ' + str(self.child_frame) + 'found.')
        
        ############
        ##Init end##
        ############

    def reduce_mixture(self, gated_hypotheses, gated_weights):
        """
        Reduces a Gaussian Mixture to a single Gaussian

        Inputs:     gated_hypotheses - a list of MultiVarGauss who have been gateded with the measurement
                    gated_weights - a list of the corresponding weights which have been gated to the measurement
        
        Outputs:    reduced_hypothesis - a MultiVarGaussian

        """

        M = len(gated_weights)

        reduced_mean = 0
        cov_internal = np.zeros(np.shape(gated_hypotheses[0].cov))
        cov_external = np.zeros(np.shape(gated_hypotheses[0].cov))

        for i in range(M):
            reduced_mean += gated_hypotheses[i].mean * gated_weights[i]

        for i in range(M):
            weight_n = gated_weights[i]
            diff = np.array(gated_hypotheses[i].mean - reduced_mean)

            cov_internal += weight_n*gated_hypotheses[i].cov
            cov_external = weight_n * np.matmul(diff, diff.T)

        reduced_cov = cov_internal + cov_external

        return MultiVarGaussian(reduced_mean, reduced_cov)

    def predict_states(self):
        """
        Predicts y from x
        returns a multivariate gaussian
        """
        predicted_states = np.empty((self.active_hypotheses_count))

        for i in range(self.active_hypotheses_count):
            predicted_states[i] = self.active_hypotheses.mean
        return predicted_states
        
    def predict_measurements(self, states_pred):
        
        predicted_measurements = states_pred
        
        return predicted_measurements


    def gate_hypotheses(self, z, predicted_zs):
        
        """
        Inputs: z - MultiVarGauss of measurement
                predicted_zs - array of predicted measurement locations

        Outputs: hypothesis indices gated with measurement
        """
        gated_inds = []
        

        for i in range(self.avtive_hypotheses_count - 1):

            mahalanobis_distance = z.mahalanobis_distance_sq(predicted_zs[i])

            if mahalanobis_distance <= self.gate_size_sq:
                gated_inds.append(i)
                #m_distances.append(mahalanobis_distance)

        g = len(gated_inds)
        gated_hypotheses = [self.active_hypotheses[gated_inds[j]] for j in range(g)]
        gated_weights = np.array([self.gmm_weights[gated_inds[k]] for k in range(g)])

        
        return gated_hypotheses, gated_weights, gated_inds
    
    
    def associate_and_update(self, gated_hypotheses, gated_weights, gated_inds, z):
        
        """
        Inputs: gated_hypotheses - a list of MultiVarGauss of the active hypotheses which have been gated. 

        3 valid cases for the logic here:
            a) 1 associated hypothesis: perform a KF update with the measurement
            b) more than 1 associated hypothesis: perform a mixture reduction on associated hypotheses and update with reduced mixture
            c) 0 associated hypotheses: initiate a hypothesis with the mean of the measurement and sensor coviariance matrix

        """
        if len(gated_hypotheses) == 1:
            ass_ind = gated_inds
            self.active_hypotheses[ass_ind] = self.kf_function(z, self.active_hypotheses[ass_ind])

            self.gmm_weights[ass_ind + 1] = self.gmm_weights[ass_ind + 1] + self.boost_prob
            
        elif len(gated_hypotheses) > 1:
            
            # Perform mixture reduction on gated hypotheses to produce associated hypothesis. Sum weights, boost
            ass_weight = np.sum(gated_weights)
            ass_weight += self.boost_prob
            ass_hypothesis = self.reduce_mixture(gated_hypotheses, gated_weights)
            _, _, upd_hypothesis = self.kf_function(z, ass_hypothesis)

            # Remove the gated hypotheses from the list, replace with reduced associated hypothesis. normalize
            for ind in sorted(gated_inds, reverse = True): 
                del self.gmm_weights[ind]
                del self.active_hypotheses[ind]
            

            self.active_hypotheses.append(upd_hypothesis)
            self.gmm_weights.append(ass_weight)
            
        else:
            ass_hypothesis = z
            ass_weight = self.init_wight

            self.active_hypotheses.append(ass_weight)
            self.gmm_weights.append(ass_weight)
        
        self.gmm_weights = self.gmm_weights / sum(self.gmm_weights)
        self.active_hypotheses_count = len(self.active_hypotheses)
    
    def gmf_eval(self):
        """
        Look at active hypotheses weights and perform the following logic:
            1) If any weights above termination_criterion --> return a bool to Terminate GMF scheme
            2) If any weights under the survival_treshold --> terminate these hypotheses, normalize weights
        
        Returns:    termination_bool - True if termination criterion is reached
                    best_ind - index at highest weight value of active hypotheses
        """
        termination_bool = False

        for i in range(self.active_hypothesis_count):
            if self.gmm_weights[i] >= self.termination_criterion and i != 0:
                self.termination_bool = True

            if self.gmm_weights[i] <= self.survival_threshold and i != 0:
                del self.gmm_weights[i]
                del self.active_hypotheses[i-1]

        self.gmm_weights = self.gmm_weights / sum(self.gmm_weights)
        self.active_hypotheses_count = len(self.active_hypotheses)

        best_ind = self.gmm_weights.index(max(self.gmm_weights))
        
        return termination_bool, best_ind
        
                
    def update_mission(self, mission):
        self.mission_topic = mission.data

    def get_Ts(self):
        Ts = rospy.get_time() - self.last_time
        return Ts
    
    def kf_function(self, z, ass_hypothesis):

        Ts = self.get_Ts()

        gauss_x_pred, gauss_z_pred, gauss_est = self.my_ekf.step_with_info(ass_hypothesis, z, Ts)
        
        self.last_time = rospy.get_time()

        return gauss_x_pred, gauss_z_pred, gauss_est

    def est_to_pose(self, x_hat):
        x = x_hat[0]
        y = x_hat[1]
        z = x_hat[2]
        pos = [x, y, z]

        euler_angs = [x_hat[3], x_hat[4], x_hat[5]]
        return pos, euler_angs

    def transformbroadcast(self, parent_frame, p):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = parent_frame
        t.child_frame_id = "object_" + str(p.objectID)
        t.transform.translation.x = p.objectPose.pose.position.x
        t.transform.translation.y = p.objectPose.pose.position.y
        t.transform.translation.z = p.objectPose.pose.position.z
        t.transform.rotation.x = p.objectPose.pose.orientation.x
        t.transform.rotation.y = p.objectPose.pose.orientation.y
        t.transform.rotation.z = p.objectPose.pose.orientation.z
        t.transform.rotation.w = p.objectPose.pose.orientation.w
        self.__tfBroadcaster.sendTransform(t)


    def publish_function(self, objectID, best_position, best_pose_quaterion, termination_boolean):

        ## TODO: Figure out how to publish the termination boolean
        ## TODO: WITH FINN AND TAREK: figure out what we are going to publish here, and then implement it.
        p = ObjectPosition()
        #p.pose.header[]
        p.objectID = objectID
        p.objectPose.header = "object_" + str(objectID)
        p.objectPose.pose.position.x = best_position[0]
        p.objectPose.pose.position.y = best_position[1]
        p.objectPose.pose.position.z = best_position[2]
        p.objectPose.pose.orientation.x = best_pose_quaterion[0]
        p.objectPose.pose.orientation.y = best_pose_quaterion[1]
        p.objectPose.pose.orientation.z = best_pose_quaterion[2]
        p.objectPose.pose.orientation.w = best_pose_quaterion[3]
        
        self.gate_pose_pub.publish(p)
        rospy.loginfo("Object published: %s", objectID)
        self.transformbroadcast(self.parent_frame, p)

    def obj_pose_callback(self, msg):

        rospy.loginfo("Object data recieved for: %s", msg.objectID)
        self.current_object = msg.objectID
        
        # Figure this out later
        #if self.mission_topic == self.current_object + "_execute":
        #    rospy.loginfo("Mission status: %s", self.mission_topic)
        #    self.prev_gauss = MultiVarGaussian(self.x_hat0, self.P_hat0)
        #    self.last_time = rospy.get_time()
        #    return None
        

        # Extract measurement of object from message
        obj_pose_position_wg = np.array([msg.objectPose.pose.position.x, 
                                         msg.objectPose.pose.position.y, 
                                         msg.objectPose.pose.position.z])

        obj_pose_pose_wg = np.array([msg.objectPose.pose.orientation.x,
                                     msg.objectPose.pose.orientation.y,
                                     msg.objectPose.pose.orientation.z,
                                     msg.objectPose.pose.orientation.w])

        # Prepare measurement vector
        z_phi, z_theta, z_psi = tft.euler_from_quaternion(obj_pose_pose_wg, axes='sxyz')

        z = obj_pose_position_wg
        z = np.append(z, [z_phi, z_theta, z_psi])
        z_gauss = MultiVarGaussian(z, self.R)
        
        # Figure this out later, initialization stuff
        #if sorted(self.prev_gauss.mean) == sorted(self.x_hat0):
        #    self.prev_gauss = MultiVarGaussian(z, self.P_hat0)
        #    self.last_time = rospy.get_time()
        #    return None

        pred_states = self.predict_states()
        pred_zs = self.predict_measurements(pred_states)

        gated_hypotheses, gated_weights, gated_inds = self.gate_hypotheses(z_gauss, pred_zs)

        self.associate_and_update(gated_hypotheses, gated_weights, gated_inds)
        self.gmf_eval()

        termination_bool, best_ind = self.gmf_eval()

        if best_ind == 0:
            best_hypothesis = self.null_gauss
        else:
            best_hypothesis = self.active_hypotheses[best_ind - 1]

        x_hat_best = best_hypothesis.mean
        best_position, best_pose = self.est_to_pose(x_hat_best)
        best_pose_quaternion = tft.quaternion_from_euler(best_pose[0], best_pose[1], best_pose[2])
        
        self.publish_function(msg.objectID, best_position, best_pose_quaternion, termination_bool)


if __name__ == '__main__':
    while not rospy.is_shutdown():     
        try:
            gmm_filter = GMMFNode()
            rospy.spin()
        except rospy.ROSInterruptException:
            pass
    
