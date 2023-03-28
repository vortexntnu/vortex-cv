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
#import dynamic_reconfigure.client

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
from image_extraction import Image_extraction
from RANSAC import RANSAC, LinearRegressor
from sympy import symbols, Eq, solve
"""
Node made to publish data to the landmarkserver of type "Objectposition", which is an own defined Vortex msg and can be found in the vortex-msgs respository.
It takes in data from the UDFC (Monocamera facing downwards under Beluga). The "mission_topic_sub"-subscriber controls if our node is running.
It is running if the state = "path/search", "path/converge" or "path/execute".
For the TAC 2023, the execute state will be the most relevant, but we dont have to get as good data to estimate direction as they did in the 2022 Robosub.
This node is mainly a hard copy of thepath_following_node, used at Robosub 2022, but with adaptions for the TAC2023"
Tuva  
"""


class PipelineFollowingNode():
    """
    Node for comperhensive solution to the path following task in RoboSub 2022 on Perception side
    The Node only operates when fsm is in path_search/converge/execute state, and governs the transition between these
    
    3 main problems need to be solved by the node, and they are associated to the possible states for path:

    Doesnt currently make any difference which state we are in

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
        self.parent_frame = 'base_link' 
        self.child_frame = 'udfc_link'

        fx_opt, fy_opt, cx_opt, cy_opt = rospy.get_param(
            "fx_opt"), rospy.get_param("fy_opt"), rospy.get_param(
                "cx_opt"), rospy.get_param("cy_opt")
        self.K_opt = np.eye(3)
        self.K_opt[0, 0] = fx_opt
        self.K_opt[1, 1] = fy_opt
        self.K_opt[0, 2] = cx_opt
        self.K_opt[1, 2] = cy_opt

        self.K_opt_inv = np.linalg.inv(self.K_opt)

        #Parameters for pipe color (lower-> more red, higher -> more green, yellow is around 60)
        self.lower_hue = 20
        self.upper_hue = 80

        # Subscribers
        self.udfcSub = rospy.Subscriber("/cv/image_preprocessing/CLAHE/udfc",
                                        Image, self.path_following_udfc_cb)
        self.mission_topic_sub = rospy.Subscriber("/fsm/state", String,
                                                  self.update_mission)

        #Topic to be subscribed by the landmark server
        self.wpPub = rospy.Publisher('/fsm/object_positions_in',
                                     ObjectPosition,
                                     queue_size=1)

        # Initialize state and bools
        self.possible_states = ["path/search", "path/converge", "path/execute"]
        self.current_state = ""
        self.objectID = "pipeline"
        self.detection_area_threshold = 200

        self.isDetected = False
        self.estimateConverged = False
        self.estimateFucked = False

        self.batch_line_params = np.zeros((1, 3))

        # TODO: find realistic values for these. Needs to be field tested.
        # Huseby 20.07.2022 patch: these should now be useless!!!
        self.H_pool_prior = 2  # Depth of pool
        self.h_path_prior = 0.1  # Height of path
        self.Z_prior_ref = 1  # Optimal z-coordinate for performing pfps,will be regulated for TAC

        self.extractor = Image_extraction()

        self.__tfBuffer = tf2_ros.Buffer()
        self.__listener = tf2_ros.TransformListener(self.__tfBuffer)
        self.__tfBroadcaster = tf2_ros.TransformBroadcaster()

        #The init will only continue if a transform between parent frame and child frame can be found
        while self.__tfBuffer.can_transform(self.parent_frame, self.child_frame, rospy.Time()) == 0:
            try:
                rospy.loginfo("No transform between " +
                              str(self.parent_frame) + ' and ' +
                              str(self.child_frame))
                rospy.sleep(2)
            except:  #, tf2_ros.ExtrapolationException  (tf2_ros.LookupException, tf2_ros.ConnectivityException)
                rospy.sleep(2)
                continue

        rospy.loginfo("Transform between " + str(self.parent_frame) + ' and ' +
                      str(self.child_frame) + 'found.')

        # Wait for first image
        self.bridge = CvBridge()
        img = rospy.wait_for_message("/cv/image_preprocessing/CLAHE_single/udfc", Image)
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img, "passthrough")
            self.image_shape = cv_image.shape
            self.img_center = np.array(
                [self.image_shape[0] / 2, self.image_shape[1] / 2])

        except CvBridgeError:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def publish_waypoint(self, publisher, objectID, waypoint):
        """
        Publishes a waypoint as an Objectposition-msg, using the given publisher. The interesting components are the Object Id, x and y positions and whether anything is detected or not.
        """

        #P is the object to be sent on the "/fsm/state"-topic, ObjectPosition is a Vortex-defined msg.
        p = ObjectPosition()
        #p.pose.header[]
        p.objectID = objectID

        p.objectPose.pose.position.x = waypoint[0]
        p.objectPose.pose.position.y = waypoint[1]
        p.objectPose.pose.position.z = waypoint[2]  #z is set to zero as default, we need to ignore depth data comming from this class.
        p.objectPose.pose.orientation.x = 0
        p.objectPose.pose.orientation.y = 0
        p.objectPose.pose.orientation.z = 0
        p.objectPose.pose.orientation.w = 1

        p.isDetected = self.isDetected
        p.estimateConverged = self.estimateConverged
        p.estimateFucked = self.estimateFucked

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

        z_over_path = self.H_pool_prior - abs(
            trans_udfc_odom[2]) - self.h_path_prior

        # Map X and Y to world frame
        X = z_over_path * point_cam[0]
        Y = z_over_path * point_cam[1]

        if dp_ref:
            point_odom = np.array([X, Y, self.Z_prior_ref]) + trans_udfc_odom
        else:
            point_odom = np.array([X, Y, z_over_path]) + trans_udfc_odom
        return point_odom

    def find_line(self, contour):
        """
        Uses RANSAC to find line and returns direction vector "colin_vec"
        and start point "p0"
        """
        points = np.argwhere(contour > 1)
        X = points[:, 0].reshape(-1, 1)
        y = points[:, 1].reshape(-1, 1)

        def square_error_loss(y_true, y_pred):
            return (y_true - y_pred)**2

        def mean_square_error(y_true, y_pred):
            return np.sum(square_error_loss(y_true, y_pred)) / y_true.shape[0]

        n = 5
        k = 1000
        t = 10000
        d = np.size(points)/10
        regressor = RANSAC(n,
                           k,
                           t,
                           d,
                           model=LinearRegressor(),
                           loss=square_error_loss,
                           metric=mean_square_error)

        regressor.fit(X, y)

        params = regressor.best_fit.params
        alpha = float(params[1])
        beta = float(params[0])
        x, y = symbols('x y')

        y = alpha * x + beta
        sol = solve(Eq(y,620))

        vx = 1
        vy = alpha

        colin_vec = np.ravel(np.array((vx, vy)))
        colin_vec /= np.linalg.norm(colin_vec)
        p0 = [620,float(sol[0])]
        print('colin_vec:',colin_vec)
        print('p0:', p0)

        return colin_vec, p0

    def estimate_next_waypoint(self, t_udfc_odom, colin_vec, p0):
        """
        Not ftested in any way!
        Brainstorming:
        -Assuming we have line fromfind line-function.
        -Needs to be found from current position
        -Cameraframe: line
        -Odomframe: current position, output position
        -Output:  waypoint, 3 state array, z = 0


        """
        #Getting the point
        p1 = p0 - 1000 * np.abs(colin_vec)
        print('p1: ', p1)
        waypoint = self.map_to_odom(p1, t_udfc_odom)
        print('waypoint: ', waypoint)
        return waypoint

    def findContour(self, img):

        contour = self.extractor.YellowEdgesHSV(img, self.lower_hue,
                                                self.upper_hue)
        points = np.argwhere(contour > 1)
        print('Number of points in contour: ', points[:,0].size)

        if points[:,0].size > self.detection_area_threshold:
            print('isDetected = True')
            self.isDetected = True
            return contour
        else:
            print('isDetected = False')
            self.isDetected = False
            return None

    #The function to rule them  all
    def path_following_udfc_cb(self, img_msg):
        """Callbackfunction for the camera subscriber """

        self.waypoint_header = img_msg.header

        #Killing the  node if it is not in its operating state(Given by the state machine-Subscribernode).
        # if self.current_state not in self.possible_states:
        #     return None

        #Convertes from camera coordinates to odom/Beluga coordinates
        tf_lookup_wc = self.__tfBuffer.lookup_transform(
            self.parent_frame, self.child_frame, rospy.Time(),
            rospy.Duration(5))
        t_udfc_odom = np.array([
            tf_lookup_wc.transform.translation.x,
            tf_lookup_wc.transform.translation.y,
            tf_lookup_wc.transform.translation.z
        ])

        udfc_img = self.bridge.imgmsg_to_cv2(img_msg, "passthrough")

        # Extracting the contour
        contour = self.findContour(udfc_img)
        
        if contour is not None:
            #Approximating the line
            colin_vec, p0 = self.find_line(contour)

            #Estimating the next waypoint
            waypoint = self.estimate_next_waypoint(t_udfc_odom, colin_vec, p0)

            #Publishing the next waypoint
            self.publish_waypoint(self.wpPub, "Pipeline", waypoint)


if __name__ == '__main__':
    try:
        pipeline_following_node = PipelineFollowingNode()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
