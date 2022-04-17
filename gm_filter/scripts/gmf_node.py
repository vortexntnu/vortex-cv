#!/usr/bin/env python

#import debugpy
#print("Waiting for VSCode debugger...")
#debugpy.listen(5678)
#debugpy.wait_for_client()

##EKF imports
from ast import Mult
from re import X

from ekf_python2.gaussparams_py2 import MultiVarGaussian
from ekf_python2.dynamicmodels_py2 import landmark_pose_world
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

class GMFNode:
    

    def __init__(self):
        ########################################
        ####Things you can change yourself####
        ########################################

        #Name of the node
        node_name = "gmf"

        #Frame names, e.g. "odom" and "cam"

        ## TODO: fix after talk with Finn and Tarek
        # Don't think there is much to fix here tbh

        self.parent_frame = 'odom' 
        self.child_frame = 'zed2_left_camera_frame'
        self.object_frame = ""

        self.current_object = ""

        #Subscribe topic
        # TODO: this is just like object_edc, which was used on pool test, but why?? Is everything a spy??
        object_topic_subscribe = "/pointcloud_processing/object_pose/spy"
        mission_topic_subscribe = "/fsm/state"

        ##################
        #### GMF stuff ###
        ##################


        # Tuning Parameters for the GMF scheme:
        self.init_prob = 0.2
        self.boost_prob = 0.1
        self.termination_criterion = 0.9
        self.survival_threshold = 0.15
        gate_percentile = 0.15
        self.max_nr_hypotheses = 25
        # TODO: find a good number for these by looking at how many iterrations it takes to get that low
        self.covariance_norm_convergence = 1e-6
        self.max_kf_iterrations = 30


        # Measurement gate
        ndim = 6
        self.gate_size_sq = chi2.ppf(gate_percentile, ndim)

        # Weights and hypothesis init
        self.best_ind = 0
        self.gmf_weights = np.array([1])
        self.active_hypotheses = []
        self.active_hypotheses_count = 0
        self.nr_of_kf_updates = 0

        # Initialize GMF termination bool, which is isDetected bool for Autonomous
        self.termination_bool = False
        self.estimateConverged = False
        self.estimateFucked = False

        ##################
        ###  EKF stuff ###
        ##################

        # Tuning parameters
        self.sigma_a = 0*np.array([0.05, 0.05, 0.05, 0.05, 0.05, 0.05])
        self.sigma_z = 2*np.array([0.5, 0.5, 0.5, 0.5, 0.5, 0.5])

        # Making filtering objects: dynamic model, measurement model and ekf
        self.landmark_model = landmark_pose_world(self.sigma_a)
        self.sensor_model = measurement_linear_landmark(self.sigma_z)
        self.my_ekf = EKF(self.landmark_model, self.sensor_model)

        # Initialization covariance for a new hypothesis
        init_vars = np.ones((6,))
        self.P_hat0 = 100 * np.diag(init_vars)

        ################
        ###ROS stuff####
        ################

        # ROS node init
        rospy.init_node(node_name)
        self.last_time = rospy.get_time()

        now = rospy.get_rostime()
        rospy.loginfo("Current time %i %i", now.secs, now.nsecs)

        # Subscribe to mission topic
        # TODO: figure out what we do here with Tarek and Finn
        # We always start in _search state. This will later be updated from first measurement arrival
        self.mission_topic = self.current_object + "_search"
        self.mission_topic_sub = rospy.Subscriber(mission_topic_subscribe, String, self.update_mission)

        # Subscriber to gate pose and orientation 
        self.object_pose_sub = rospy.Subscriber(object_topic_subscribe, ObjectPosition, self.gmf_callback, queue_size=1)
      
        # Publisher to Autonomous
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

    def reduce_mixture(self, gated_hypotheses, gated_weights):
        # Tested, works
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
        returns a list of predicted means
        """
        predicted_states = np.empty((self.active_hypotheses_count,))

        for i in range(self.active_hypotheses_count):
            predicted_states[i] = self.active_hypotheses.mean
        return predicted_states
        
    def predict_measurements(self, states_pred):
        
        predicted_measurements = states_pred
        
        return predicted_measurements


    def gate_hypotheses(self, z_gauss, predicted_zs):
        # Tested, works

        """
        Inputs: z - MultiVarGauss of measurement
                predicted_zs - array of predicted measurement locations

        Outputs: hypothesis indices gated with measurement
        """
        gated_inds = []


        for i in range(self.active_hypotheses_count):
            mahalanobis_distance = z_gauss.mahalanobis_distance_sq(predicted_zs[i])
            if mahalanobis_distance <= self.gate_size_sq:
                gated_inds.append(i)
                #m_distances.append(mahalanobis_distance)
        g = len(gated_inds)
        gated_hypotheses = [self.active_hypotheses[gated_inds[j]] for j in range(g)]
        gated_weights = np.array([self.gmf_weights[gated_inds[k] + 1] for k in range(g)])
        
        return gated_hypotheses, gated_weights, gated_inds
    
    
    def associate_and_update(self, gated_hypotheses, gated_weights, gated_inds, z):
        # Tested, works
        
        """
        Inputs: 
            gated_hypotheses: a list of MultiVarGauss of the active hypotheses which have been gated.
            gated_weights: a list of weights associated to the active hypotheses gated
            gated_inds: a list of indices in the self.active_hypotheses variable which have been gated.
                        Notice this is NOT indices in the gmf_weights (that is, nothing here has anythin
                        to do with the null hypothesis)

        Performs logic to associate and update the active hypotheses and their weights.

        3 valid cases for the logic here:
            a) 1 gated hypothesis: associate to that hypothesis and perform a KF update with the measurement
            b) more than 1 gated hypotheses: perform a mixture reduction on gated hypotheses associate and 
            update with reduced mixture.
            c) 0 associated hypotheses: initiate a hypothesis with the mean of the measurement and P_hat0 cov
            matrix
        """
        if len(gated_hypotheses) == 1:
            ass_ind = gated_inds[0]
            _, _, upd_hypothesis = self.kf_function(z, self.active_hypotheses[ass_ind])
            self.active_hypotheses[ass_ind] = upd_hypothesis

            self.gmf_weights[ass_ind + 1] = self.gmf_weights[ass_ind + 1] + self.boost_prob
            
        elif len(gated_hypotheses) > 1:
            
            # Perform mixture reduction on gated hypotheses to produce associated hypothesis. Sum weights, boost
            ass_weight = np.sum(gated_weights)
            ass_weight += self.boost_prob
            ass_hypothesis = self.reduce_mixture(gated_hypotheses, gated_weights)
            _, _, upd_hypothesis = self.kf_function(z, ass_hypothesis)

            # Remove the gated hypotheses from the list, replace with reduced associated hypothesis. normalize
            for ind in sorted(gated_inds, reverse = True): 
                self.gmf_weights = np.delete(self.gmf_weights, ind + 1)
                del self.active_hypotheses[ind]
            
            self.active_hypotheses.append(upd_hypothesis)
            self.gmf_weights = np.append(self.gmf_weights, ass_weight)

            
        else:
            # In case of no gated hypotheses initiate a new KF with mean of measurement
            ass_hypothesis = MultiVarGaussian(z, self.P_hat0)
            ass_weight = self.init_prob

            self.active_hypotheses.append(ass_hypothesis)
            self.gmf_weights = np.append(self.gmf_weights, ass_weight)
        
        self.gmf_weights = self.gmf_weights / sum(self.gmf_weights)
        self.active_hypotheses_count = len(self.active_hypotheses)

    def gmf_eval(self):
        # Tested, works
        """
        Look at active hypotheses weights and perform the following logic:
            0) If there are no active hypotheses --> exit this function
            1) If any weights above termination_criterion --> return a bool to terminate GMF scheme
            2) If any weights under the survival_treshold --> terminate these hypotheses, normalize weights
        
        Returns:    termination_bool - True if termination criterion is reached
                    best_ind - index at highest weight value of active hypotheses
        """
        nr_remove = 15

        if self.active_hypotheses_count == 0:
            self.gmf_weights = self.gmf_weights / sum(self.gmf_weights)
            return None

        remove_inds = []
        for i in range(len(self.gmf_weights)):
            if self.gmf_weights[i] >= self.termination_criterion and i != 0:
                self.termination_bool = True

            if self.gmf_weights[i] <= self.survival_threshold and i != 0:
                remove_inds.append(i)
                
        
        for remove_ind in sorted(remove_inds, reverse=True):
            self.gmf_weights = np.delete(self.gmf_weights, remove_ind)
            del self.active_hypotheses[remove_ind - 1]

        if len(self.active_hypotheses) >= self.max_nr_hypotheses:
            # Find the smallest 15 values, sum them up and add them to the null hypothesis

            hypotheses_weights = self.gmf_weights.copy()
            hypotheses_weights = np.delete(hypotheses_weights, 0)

            k_smallest_inds = np.argpartition(hypotheses_weights, nr_remove)
            termination_weights = hypotheses_weights[k_smallest_inds[:nr_remove]]
            new_null_weight = self.gmf_weights[0] + sum(termination_weights)

            termination_weights = np.delete(hypotheses_weights, k_smallest_inds[:nr_remove])
            termination_weights = np.concatenate(([new_null_weight], termination_weights))
            self.gmf_weights = termination_weights
            

        self.gmf_weights = self.gmf_weights / sum(self.gmf_weights)
        self.active_hypotheses_count = len(self.active_hypotheses)

        self.best_ind = np.argmax(self.gmf_weights)

        rospy.loginfo("Number of active hypotheses is now: %s", self.active_hypotheses_count)
        rospy.loginfo("Highest probability for a hypothesis is: %s", np.max(self.gmf_weights))
        rospy.loginfo("Most likely hypothesis is: %s", self.best_ind)
    
    
    
    def gmf_reset(self):
        """
        Resets the GMF variables to what they are in the init function. This happens on transition from converge to search or from execute
        to search.

        """
        # Reset variables
        self.best_ind = 0
        self.gmf_weights = np.array([1])
        self.active_hypotheses = []
        self.active_hypotheses_count = 0
        self.nr_of_kf_updates = 0
        
        # Reset booleans
        self.termination_bool = False
        self.estimateConverged = False
        self.estimateFucked = False

    def evaluate_filter_convergence(self, objectID):
        # Not tested, but should work :D
        """
        Computes the Frobenious norm of the covariance matrix and checks if it is under the filter convergence criterion. Updates the
        convergenceFucked boolean accordingly and publishes.
        """
        Fnorm_P = np.linalg.norm(self.best_hypothesis.cov, "fro")
        self.nr_of_kf_updates = self.nr_of_kf_updates + 1
        
        # TODO: simulate what it takes for the F norm to reach certain values
        if Fnorm_P <= self.covariance_norm_convergence:
            # We tell Autonomous to execute
            self.estimateConverged = True
            self.publish_function(objectID)
            return None

        elif self.nr_of_kf_updates >= self.max_kf_iterrations:
            # We tell Autonomous to go back to search
            self.estimateFucked = True
            self.publish_function(objectID)
            return None
        else:
            pass

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


    def publish_function(self, objectID):
        
        x_hat_best = self.best_hypothesis.mean
        best_position, best_pose = self.est_to_pose(x_hat_best)
        best_pose_quaternion = tft.quaternion_from_euler(best_pose[0], best_pose[1], best_pose[2])
        
        p = ObjectPosition()
        #p.pose.header[]
        p.objectID = objectID
        p.objectPose.header = "object_" + str(objectID)
        p.objectPose.pose.position.x = best_position[0]
        p.objectPose.pose.position.y = best_position[1]
        p.objectPose.pose.position.z = best_position[2]
        
        p.objectPose.pose.orientation.x = best_pose_quaternion[0]
        p.objectPose.pose.orientation.y = best_pose_quaternion[1]
        p.objectPose.pose.orientation.z = best_pose_quaternion[2]
        p.objectPose.pose.orientation.w = best_pose_quaternion[3]

        p.isDetected = self.termination_bool
        #p.estimateConverged = False
        #p.estimateFucked = False
        
        self.gate_pose_pub.publish(p)
        rospy.loginfo("Object published: %s", objectID)
        self.transformbroadcast(self.parent_frame, p)
        
    def gmf_callback(self, msg):
        # TODO: read through carefully and follow logic

        #rospy.loginfo("Object data recieved for: %s", msg.objectID)
        # Should not need if fsm works well: 
        self.old_object = self.current_object.copy()
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
        
        # In the case of failed convergence validation from the VKF, we will go from current_object + "converge" to current_object + "search"
        current_action = self.mission_topic.split("_")[1]
        old_action = self.mission_topic_old.split("_")[1]
        
        # Reset GMF on going from converge back to search
        if old_action == "converge" and current_action == "search":
            self.gmf_reset()
            self.mission_topic_old = self.mission_topic.copy()
            return None

        # Reset GMF on going from either execute or converge to search
        elif old_action == "execute" and current_action == "search":
            self.gmf_reset()
            self.mission_topic_old = self.mission_topic.copy()
            return None
        else:
            pass

        # Upon GMF convergence, update only the best result and publish that
        if self.termination_bool:
            _, _, self.best_hypothesis = self.kf_function(z, self.best_hypothesis)
            self.publish_function(msg.objectID)
            rospy.loginfo("GMF converged at an estimate with nr of active hypotheses: %s", self.active_hypotheses_count)
            self.mission_topic_old = self.mission_topic[:]
            self.evaluate_filter_convergence(msg.objectID)
            return None

        # Initialize with first measurement
        if len(self.active_hypotheses) == 0:
            self.active_hypotheses.append(MultiVarGaussian(z_gauss.mean, self.P_hat0))

            self.gmf_weights = np.append(self.gmf_weights, self.init_prob)
            self.gmf_weights = self.gmf_weights / sum(self.gmf_weights)
            self.active_hypotheses_count = len(self.active_hypotheses)

            self.last_time = rospy.get_time()
            return None

        pred_states = self.predict_states()
        pred_zs = self.predict_measurements(pred_states)

        gated_hypotheses, gated_weights, gated_inds = self.gate_hypotheses(z_gauss, pred_zs)

        self.associate_and_update(gated_hypotheses, gated_weights, gated_inds, z)
        self.gmf_eval()

        if self.best_ind == 0:
            return None
        else:
            self.best_hypothesis = self.active_hypotheses[self.best_ind - 1]

        self.publish_function(msg.objectID)
        #self.publish_function_test(msg.objectID)
        self.mission_topic_old = self.mission_topic[:]


if __name__ == '__main__':
    while not rospy.is_shutdown():     
        try:
            gm_filter = GMFNode()
            rospy.spin()
        except rospy.ROSInterruptException:
            pass
    
