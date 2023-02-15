#!/usr/bin/env python

from cProfile import label
from matplotlib.pyplot import contour
from plumbum import local
import rospy
import rospkg

from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Empty, String
from cv_msgs.msg import Point2, PointArray
#from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes

from cv_bridge import CvBridge, CvBridgeError
import dynamic_reconfigure.client

#from geometry_msgs.msg import PointStamped
from vortex_msgs.msg import ObjectPosition
from geometry_msgs.msg import TransformStamped, Point

import tf2_ros
import tf2_geometry_msgs.tf2_geometry_msgs

import numpy as np
import matplotlib.pyplot as plt
from timeit import default_timer as timer
import traceback
import cv2
from os.path import join

import feature_detection

"""
Node made to publish data to the landmarkserver of type "Objectposition", which is an own defined Vortex msg and can be found in the vortex-msgs respository.
It takes in data from the UDFC (Monocamera facing downwards under Beluga). The "mission_topic_sub"-subscriber controls if our node is running.
It is running if the state = "path/search", "path/converge" or "path/execute".
For the TAC 2023, the execute state will be the most relevant, but we dont have to get as good data to estimate direction as they did in the 2022 Robosub.
This node is mainly a hard copy of thepath_following_node, used at Robosub 2022, but with adaptions for the  TAC2023"
Tuva  
"""

class PipelineFollowingNode():
    """
    Node for comperhensive solution to the path following task in RoboSub 2022 on Perception side
    The Node only operates when fsm is in path_search/converge/execute state, and governs the transition between these
    
    3 main problems need to be solved by the node, and they are associated to the possible states for path:

        path_search:        During path search the node uses ZED camera feed to detect the path. After path is detected, the coordinates are published to Autonomous
                            and we request switch to converge.

        path_converge:      During path converge the goal is to center the path in the FOV of the underwater downwards facing camera (in the future refered to as the UDFC)
                            The path is classified in UDFC frame and it's areal centroid is regulated to the middle of the image through a local PID scheme. The assumption being
                            that this scheme will progressively get the path more and more into FOV, and finally in the centre of UDFC frame. After the path is sufficiently
                            centered we move on to execute.

        path_execute:       During execute the drone keeps station over the path while Perception solves the estimation problem of the path to the next task. Here we can use
                            the fact that we are given the oportunity for multiple measurements and samples to cast the task as a batch problem and extract as much information
                            as possible before we declare a solution. The search for the next task starts when a waypoint is given to Autonomous after we have estimated path 
                            direction.
    """

    def __init__(self):
        rospy.init_node('pointcloud_processing_node')

        # For publishing results for Auto, parameters for converging data to odor frame. 
        self.parent_frame = 'odom' 
        self.child_frame = 'udfc_link'
        
        fx_opt, fy_opt, cx_opt, cy_opt = rospy.get_param("fx_opt"), rospy.get_param("fy_opt"), rospy.get_param("cx_opt"), rospy.get_param("cy_opt")
        self.K_opt = np.eye(3)
        self.K_opt[0,0] = fx_opt
        self.K_opt[1,1] = fy_opt
        self.K_opt[0,2] = cx_opt
        self.K_opt[1,2] = cy_opt

        self.K_opt_inv = np.linalg.inv(self.K_opt)

        # Parameters for the algorithm:
        self.hsv_params = [0,       #hsv_hue_min
                           53,     #hsv_hue_max
                           71,       #hsv_sat_min
                           131,     #hsv_sat_max
                           175,       #hsv_val_min
                           248]     #hsv_val_max
    
        self.ksize1     = 7
        self.ksize2     = 7
        self.sigma      = 0.8
        
        # Thresholding params, consider adapting to match physical condition
        self.thresholding_blocksize     = 11
        self.thresholding_C             = 2

        # Erosion and dilation params, consider adapting to match physical condition,we probably have more erosion
        self.erosion_dilation_ksize     = 5
        self.erosion_iterations         = 1
        self.dilation_iterations        = 1
        self.noise_rm_params = [self.ksize1, self.ksize2, self.sigma, self.thresholding_blocksize, self.thresholding_C, self.erosion_dilation_ksize, self.erosion_iterations, self.dilation_iterations]

        # Subscribers
        self.udfcSub = rospy.Subscriber("/cv/image_preprocessing/CLAHE/udfc", Image, self.path_following_udfc_cb)
        self.mission_topic_sub = rospy.Subscriber("/fsm/state", String, self.update_mission)

        #Topic to be subscribed by the landmark server
        self.wpPub = rospy.Publisher('/fsm/object_positions_in', ObjectPosition, queue_size=1)

        # Initialize state and bools
        self.possible_states = ["path/search", "path/converge", "path/execute"]
        self.current_state = ""
        self.objectID = "path"
        self.detection_area_threshold = 2000

        self.isDetected            = False
        self.estimateConverged     = False
        self.estimateFucked        = False

        self.batch_line_params = np.zeros((1,3))
        self.uppest_y_coords = []

        # TODO: find realistic values for these. Needs to be field tested.
        # Huseby 20.07.2022 patch: these should now be useless!!!
        self.H_pool_prior       = 2     # Depth of pool
        self.h_path_prior       = 0.1   # Height of path
        self.Z_prior_ref        = 1  # Optimal z-coordinate for performing pfps,will be regulated for TAC 

        # Wait for first image
        img = rospy.wait_for_message("/cv/image_preprocessing/CLAHE_single/udfc", Image)
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img, "passthrough")
            self.image_shape = cv_image.shape
            self.img_center = np.array([self.image_shape[0]/2, self.image_shape[1]/2])

        except CvBridgeError:
            rospy.logerr("CvBridge Error: {0}".format(e))

        self.__tfBuffer = tf2_ros.Buffer()
        """
        Here is some objects for classes and transformations stuffs, consider necesity, you probably need it.
        Have also skipped some subscribers
        """

        """
        Functions from the OG, not yet included:
        cv_image_publisher
        publish_error
        local_path_errors
        dynam_reconfigure_callback - sets  new parameters  and thresholds
        """

    def publish_waypoint(self, publisher, objectID, waypoint):
        """
        Publishes one waypoint as an Objectposition-msg, using the given publisher. Object ID is part of the msg. 
        """

        #P is the object to be sent on the "/fsm/state"-topic, ObjectPosition is a Vortex-defined msg. 
        p = ObjectPosition()
        #p.pose.header[]
        p.objectID = objectID

        p.objectPose.pose.position.x = waypoint[0]
        p.objectPose.pose.position.y = waypoint[1]
        p.objectPose.pose.position.z = waypoint[2]
        p.objectPose.pose.orientation.x = 0
        p.objectPose.pose.orientation.y = 0
        p.objectPose.pose.orientation.z = 0
        p.objectPose.pose.orientation.w = 1

        p.isDetected            = self.isDetected
        p.estimateConverged     = self.estimateConverged
        p.estimateFucked        = self.estimateFucked
        
        # When we publish the buoy we set everything to false again
        if objectID == "buoy":
            p.isDetected            = False
            p.estimateConverged     = False
            p.estimateFucked        = False

        #Function-call, takes in a publisher as a variable.
        publisher.publish(p)

        rospy.loginfo("Object published: %s", objectID)

    def update_mission(self, mission):
        """
        Updates current_state 
        """
        self.current_state = mission.data

    def map_to_odom(self, point_cam, trans_udfc_odom, dp_ref=None):

        """
        Takes in a point in homogenious camera coordinates and calculates the X, Y, Z in odom frame using the similarity 
        triangles equations through the dp assumptions of no roll and pitch and known height over ground.

        Input: 
            point_cam - [x_tilde, y_tilde, 1]^T
            trans_udfc_odom - position of udfc in odom

        Output:
            point_odom - estimate of point in odom frame [X, Y, Z]^T
        """

        z_over_path = self.H_pool_prior - abs(trans_udfc_odom[2]) - self.h_path_prior
        
        # Map X and Y to world frame
        X = z_over_path * point_cam[0]
        Y = z_over_path * point_cam[1]

        if dp_ref:
            point_odom = np.array([X, Y, self.Z_prior_ref]) + trans_udfc_odom
        else:
            point_odom = np.array([X, Y, z_over_path]) + trans_udfc_odom
        return point_odom

    def find_contour_line(self, contour, img_drawn=None):
        """
        Finds the line in image coordinates, and computes two points that are later to be stored in world.

        Output:         p0 - a point on the line
                        p1 - another point in the line, exactly [vx, vy] along the line from p0, s.t. colin_vec = p_line - p0
        """
        #Figure out where  the contour comes from when called
        #Use the  fitLine() function, that gives back the vectorcoordinates vx and vy accumulated from the contours. and the startpoints y0 and x0.
        vx, vy, x0, y0 = cv2.fitLine(contour, cv2.DIST_L2, 0, 0.1, 0.1)

        colin_vec   = np.ravel(np.array((vx, vy)))
        p0          = np.ravel(np.array((x0, y0)))
        p_line      = p0 + colin_vec
        
        #draws a line through the vector, across the image.
        if img_drawn is not None:
            p1 = p0 + 1000*colin_vec
            p2 = p0 - 1000*colin_vec
            cv2.line(img_drawn, tuple(p1), tuple(p2), color=(0, 255, 0), thickness=2)

            return p_line, p0, img_drawn
        
        else:
            return p_line, p0

    def batch_estimate_waypoint(self, t_udfc_odom):
        
        uppestest_y_coord = np.average(self.uppest_y_coords, 0)
        vx, vy, x0, y0 = cv2.fitLine(self.batch_line_params[:,:2], cv2.DIST_L2, 0, 0.1, 0.1)
        
        colin_vec   = np.ravel(np.array((vx, vy)))
        p0          = np.ravel(np.array((x0, y0)))
        
        p1 = np.append(p0 + 50*colin_vec, 1)
        p2 = np.append(p0 - 50*colin_vec, 1)
        
        #Finds product between the transform matrix(default parameters) 
        p1_img = np.matmul(self.K_opt, p1 - t_udfc_odom)
        p2_img = np.matmul(self.K_opt, p2 - t_udfc_odom)


        points      = [p1, p2]
        points_img  = [p1_img, p2_img]
        
        errors          = [np.linalg.norm(uppestest_y_coord - points_img[i][:2]) for i in range(len(points_img))]
        next_waypoint   = points[np.argmin(errors)]

        #Uncomment for visualization of batch line estimation
        #fig, ax = plt.subplots()
        #ax.scatter(self.batch_line_params[:,0], self.batch_line_params[:,1], c="k", label="batch line params")
        #ax.scatter(next_waypoint[0], next_waypoint[1], c="g", label="next waypoint")
        #ax.scatter(points[np.argmax(errors)][0], points[np.argmax(errors)][1], c="r", label="wrong direction waypoint")
        #plt.legend()
        #plt.show()

        return np.append(next_waypoint, 0)

    def path_calculations(self, img):
        """
        Takes in an image and finds the path contour and centroid. Also returns the original image with these drawn.

        """
        cv2.drawContours(img, self.path_contour, -1, (0,0,255), 5)
        path_area = cv2.contourArea(self.path_contour[:,0])

        # This is just for the purposes of visualization on the image
        #Finding centroids
        M = cv2.moments(self.path_contour[:,0])
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        path_centroid = np.array([cx, cy])

        img_drawn = cv2.circle(img, (cx,cy), radius=1, color=(0, 255, 0), thickness=3)

        return path_area, img_drawn, path_centroid


    #The function to rule them  all
    def path_following_udfc_cb(self, img_msg):

        #self.waypoint_header = img_msg.header

        #Killing the  node if it is not in its operating state(Given by the state machine-Subscribernode). 
        if self.current_state not in self.possible_states:
            return None
     
        #Convertes from camera coordinates to odom/Beluga coordinates
        tf_lookup_wc    = self.__tfBuffer.lookup_transform(self.parent_frame, self.child_frame, rospy.Time(), rospy.Duration(5))
        t_udfc_odom     = np.array([tf_lookup_wc.transform.translation.x,
                                    tf_lookup_wc.transform.translation.y,
                                    tf_lookup_wc.transform.translation.z])

        #When enough data is gathered, estimates waypoint and publishes to "/fsm/state"-topic"
        #Probably have to change this part for TAC
        if np.shape(self.batch_line_params)[0] > 200:
            # In case we have gathered enough information
            next_waypoint = self.batch_estimate_waypoint(t_udfc_odom)
            self.publish_waypoint(self.wpPub, "buoy", next_waypoint)
            return None

        udfc_img = self.bridge.imgmsg_to_cv2(img_msg, "passthrough")
        
        # Extracting the contour
        _, hsv_mask, hsv_val_img = self.feature_detector.hsv_processor(udfc_img, *self.hsv_params)
        self.cv_image_publisher(self.hsvPub, hsv_val_img, msg_encoding="bgr8")

        noise_filtered_img = self.feature_detector.noise_removal_processor(hsv_mask, *self.noise_rm_params)
        self.cv_image_publisher(self.noise_filteredPub, noise_filtered_img, msg_encoding="8UC1")

        self.path_contour = self.feature_detector.contour_processing(noise_filtered_img, contour_area_threshold=3000, pfps=True, coloured_img=udfc_img, return_image=False)
        try:
            self.path_contour[:,0]
        except:
            return None

        path_area, img_drawn, self.path_centroid = self.path_calculations(udfc_img)
        self.path_contour = self.path_contour[:,0]

        # Contour detection, if area of detection is large enough, continues to next phase, consider tweeking
        if self.isDetected == False and path_area > self.detection_area_threshold:
            self.isDetected == True

        """
        This part needs adaption for pipeline following!!
        """

        # Undistort centroid point
        # These should be redundant, if the idea of local relative regulation work. Remove after that
        # has been shown to work in lab 
        #path_centroid_cam = np.matmul(self.K_opt_inv, np.append(self.path_centroid,1))
        #dp_ref = self.map_to_odom(path_centroid_cam[:2], t_udfc_odom, dp_ref=True)

        # Get the upper contour
        upper_inds = np.where((self.path_contour[:,1] < self.path_centroid[1]) == True)[0]
        upper_contour_image = self.path_contour[upper_inds]

        uppest_y_ind = np.argmin(upper_contour_image[:,1])
        self.uppest_y_coords.append(upper_contour_image[uppest_y_ind, :])
        
        # Get line, eventually as 2 points in odom and store those
        p_line, p0, img_drawn   = self.find_contour_line(upper_contour_image, img_drawn)
        p_line                  = np.append(p_line, 1)
        p0                      = np.append(p0, 1)

        # Map the points along the line to camera coordinates
        p_line_cam      = np.matmul(self.K_opt_inv, p_line)
        p0_cam          = np.matmul(self.K_opt_inv, p0)

        # Map the points along the line to odom:
        p_line_odom     = self.map_to_odom(p_line_cam, t_udfc_odom)
        p0_odom         = self.map_to_odom(p0_cam, t_udfc_odom)

        if self.current_state == "path_execute":
            uppest_y_ind = np.argmin(upper_contour_image[:,1])
            self.uppest_y_coords.append(upper_contour_image[uppest_y_ind, :])
            self.batch_line_params = np.vstack((self.batch_line_params, np.vstack((p_line_odom, p0_odom))))
        
        local_error = self.local_path_errors()

        # Publish error and drawn-on image
        self.cv_image_publisher(self.udfcPub, img_drawn, "bgr8")
        self.publish_error(local_error)

if __name__ == '__main__':
    try:
        pipeline_following_node = PipelineFollowingNode()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass