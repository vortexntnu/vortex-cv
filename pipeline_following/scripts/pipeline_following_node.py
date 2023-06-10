#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
from vortex_msgs.msg import ObjectPosition
import numpy as np
from RANSAC import RANSAC
from HOG import HOG
from matplotlib import pyplot as plt
import tf2_ros
from geometry_msgs.msg import PoseStamped
import tf2_geometry_msgs
import cv2 as cv
from tf.transformations import quaternion_from_euler
from dynamic_reconfigure.server import Server
from pipeline_following.cfg import GainTuningConfig
"""
Node for completing the pipeline following task in TAC 2023. It uses the mono-camera (UDFC) and 
publishes position data to the landmarkserver of type "ObjectPosition" (see vortex_msgs repo). 
Histogram of Orientated Gradients (HOG) is used to create  a HOG image before a auto thresholding
is performed to convert it to a binary image of the contour. Random Sample Consensus (RANSAC) is 
then used to extract a line with parameters, alpha and beta, that are used to calculate a 
vector that estimates points that will guide the drone along the pipeline.

Tuva and Lasse  
"""


class PipelineFollowingNode():

    def __init__(self):
        rospy.init_node('pipeline_following_node')

        ##### Tuning ##############################
        #Parameters for waypoint estimation
        self.K1 = -0.08  # beta error
        self.K2 = -10  # alpha error
        self.x_step = 0.5  # meters ahead of drone
        #Parameters RANSAC
        self.n = 10  # `n`: Minimum number of data points to estimate parameters
        self.k = 300  # `k`: Maximum iterations allowed
        self.t = 300  # `t`: Threshold value to determine if points are fit well
        self.d = None  # `d`: Number of close data points required to assert model fits well
        self.frac_of_points = 8  # d will be a result of the number of points in contour divided by this
        #Parameters HOG
        self.cell_size = (10, 10)  # size of each cell in the HOG descriptor
        self.block_size = (1, 1)  # size of each block in the HOG descriptor
        self.nbins = 3  # the number of orientation bins in the HOG descriptor
        #Other
        self.detection_area_threshold = 5000  # number of points in contour to accept the contour

        ###################################################

        #Subscribers
        self.udfcSub = rospy.Subscriber("/udfc/wrapper/camera_raw",
                                        Image,
                                        self.udfc_cb,
                                        queue_size=1)
        self.udfc_img = Image
        self.mission_topic_sub = rospy.Subscriber("/fsm/state", String,
                                                  self.update_mission_cb)
        self.current_state = String
        rospy.Subscriber("/odometry/filtered", Odometry, self.odom_cb)
        self.odom = Odometry

        # Publishers
        self.wpPub = rospy.Publisher('/object_positions_in',
                                     ObjectPosition,
                                     queue_size=1)
        self.wpVisualPub = rospy.Publisher('/visualize_waypoint',
                                           PoseStamped,
                                           queue_size=1)
        self.contVisualPub = rospy.Publisher('/visualize_contour',
                                             Image,
                                             queue_size=1)
        # Dynamic reconfigure

        Server(GainTuningConfig, self.TuningCallback)

        # Initialize parameters, state and bools
        self.img_size = []

        self.possible_states = ["pipeline/execute", "pipeline/standby"]
        self.current_state = ""
        self.objectID = "pipeline"

        self.isDetected = False
        self.estimateConverged = False
        self.estimateFucked = False

        # Wait for first image
        self.bridge = CvBridge()
        rospy.loginfo('Waiting for image')
        img = rospy.wait_for_message("/udfc/wrapper/camera_raw", Image)
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img, "passthrough")
            self.image_shape = cv_image.shape
            self.img_center = np.array(
                [self.image_shape[0] / 2, self.image_shape[1] / 2])

        except CvBridgeError:
            rospy.logerr("CvBridge Error: {0}".format(e))

        #TF stuff
        self.parent_frame = 'odom'
        self.child_frame = 'base_link'
        self.__tfBuffer = tf2_ros.Buffer()
        self.__listener = tf2_ros.TransformListener(self.__tfBuffer)
        self.__tfBroadcaster = tf2_ros.TransformBroadcaster()

        #The init will only continue if a transform between parent frame and child frame can be found
        while not rospy.is_shutdown() and self.__tfBuffer.can_transform(
                self.parent_frame, self.child_frame, rospy.Time()) == 0:
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

    def TuningCallback(self, config, level):

        self.K1 = config.K1  # beta error
        self.K2 = config.K2  # alpha error
        self.x_step = config.x_step  # meters ahead of drone
        self.n = config.n  # `n`: Minimum number of data points to estimate parameters
        self.k = config.k  # `k`: Maximum iterations allowed
        self.t = config.t  # `t`: Threshold value to determine if points are fit well
        self.frac_of_points = config.frac_of_points  # d will be a result of the number of points in contour divided by this
        return config

    def odom_cb(self, msg):
        """ Updates current position """
        self.odom = msg

    def update_mission_cb(self, mission):
        """ Updates current_state """
        self.current_state = mission.data

    def udfc_cb(self, img_msg):
        """ Callback for camera images """
        self.udfc_img = self.bridge.imgmsg_to_cv2(img_msg, "passthrough")
        self.image_shape = self.udfc_img.shape

    def map_to_odom(self, p_baselink, q_baselink):
        """
        Takes a pose in base_link coordinates and calculates it in odom frame.

        Input: 
            p_baselink - point in base link
            q_baselink - quaternions in base link

        Output:
            pose_odom - estimate of point/pose in odom frame
        """

        pose_base_link = PoseStamped()
        pose_base_link.header.frame_id = self.child_frame
        pose_base_link.header.stamp = rospy.Time.now()
        pose_base_link.pose.position.x = p_baselink[0]
        pose_base_link.pose.position.y = p_baselink[1]
        pose_base_link.pose.position.z = p_baselink[2]

        pose_base_link.pose.orientation.x = q_baselink[0]
        pose_base_link.pose.orientation.y = q_baselink[1]
        pose_base_link.pose.orientation.z = q_baselink[2]
        pose_base_link.pose.orientation.w = q_baselink[3]

        transform = self.__tfBuffer.lookup_transform(self.parent_frame,
                                                     self.child_frame,
                                                     rospy.Time(),
                                                     rospy.Duration(5))

        pose_odom = tf2_geometry_msgs.do_transform_pose(
            pose_base_link, transform)

        return pose_odom

    def publish_waypoint(self, publisher, objectID, pose):
        """
        input:
            publisher   - given publisher, self.wpPub
            objectID    - in this case "pipeline"
            pose        - pose of object
        """

        p = ObjectPosition()
        p.objectID = objectID
        p.objectPose.pose = pose.pose
        p.isDetected = self.isDetected
        p.estimateConverged = self.estimateConverged
        p.estimateFucked = self.estimateFucked
        publisher.publish(p)

        # cannot visualize vortex_msgs in rviz,
        # thats why this publisher are present
        p_vis = PoseStamped()
        p_vis = p.objectPose
        p_vis.header.frame_id = 'odom'
        self.wpVisualPub.publish(p_vis)

        rospy.loginfo("Object published: %s, isDetected = %s", objectID,
                      self.isDetected)

    def find_line(self, contour):
        """
        Uses RANSAC to find line and returns 
        line parameters alpha and beta

        input:
            contour - binary image containing information about the contour
        output:
            alpha   - rate of increase
            beta    - intersection with y-axis
        """

        points = np.argwhere(contour > 0)
        X = points[:, 0].reshape(-1, 1)
        y = points[:, 1].reshape(-1, 1)

        self.d = np.size(points) / self.frac_of_points
        regressor = RANSAC(self.n, self.k, self.t, self.d)

        regressor.fit(X, y)
        if regressor.fail:
            self.isDetected = False
            return None, None

        params = regressor.best_fit.params
        alpha = float(params[1])
        beta = float(params[0])

        return alpha, beta

    def estimate_next_waypoint(self, alpha, beta):
        """
        Using the line parameters to estimate next waypoint

        input:
            alpha       - rate of increase
            beta        - intersection with y-axis
        output:
            waypoint    - coordinates [x, y, z] of point in base link
            q           - quaternions [w, x, y, z] of point in base link     
        """

        # create an angle based on error
        # derived by line param alpha and beta
        e1 = self.image_shape[1] / 2 - beta
        e2 = alpha
        theta = self.K1 * e1 + self.K2 * e2
        if theta > 90:
            theta = 90
        if theta < -90:
            theta = -90
        theta_rad = theta * 2 * np.pi / 360

        # create a waypoint in front of the drone
        # based on the angle computed previously
        waypoint = self.x_step * np.array(
            [np.cos(theta_rad), np.sin(theta_rad), 0])
        q = quaternion_from_euler(0, 0, theta_rad)

        # information prints:
        # print('alpha: '+ str(alpha))
        # print('beta: '+ str(beta))
        # print('e1: '+ str(e1))
        # print('e2: '+ str(e2))
        # print('theta: '+ str(theta))
        # print('Waypoint: '+ str(waypoint))

        return waypoint, q

    def threshold(self, hog_img):
        ''' 
        Converting HOG image to binary image by thresholding
        Returning the binary image as the contour
        input:
            hog_img         - 2x2 np.array
        output:
            binary_image    - binary image of the contour 
        '''

        img_norm = cv.normalize(hog_img,
                                None,
                                alpha=0,
                                beta=255,
                                norm_type=cv.NORM_MINMAX,
                                dtype=cv.CV_8U)

        # Apply Otsu's method to calculate the threshold value
        ret, threshold = cv.threshold(img_norm, 0, 255,
                                      cv.THRESH_BINARY + cv.THRESH_OTSU)
        binary_image = np.array(threshold, dtype=np.float32) / 255

        points = np.argwhere(binary_image > 0)
        rospy.loginfo('Number of points in contour: ' + str(points[:, 0].size))

        if points[:, 0].size > self.detection_area_threshold:
            self.isDetected = True
            return binary_image
        else:
            self.isDetected = False
            return binary_image

    def findHOG(self, img):
        ''' finds HOG of the image '''
        extractor = HOG(self.cell_size, self.block_size, self.nbins)
        features, hog_img = extractor.compute_hog(img)
        return hog_img

    def plotting(self, contour, alpha=0, beta=0):
        ''' Function to visualize the computations '''
        points = np.argwhere(contour > 0)
        plt.figure(1, figsize=(10, 14))
        plt.clf()
        plt.axes().set_aspect('equal')
        plt.xlim(0, self.image_shape[0])
        plt.ylim(0, self.image_shape[1])
        plt.scatter(points[:, 0], points[:, 1])
        x = np.linspace(0, 1239, num=100).reshape(-1, 1)
        plt.plot(x, alpha * x + beta, 'r')
        plt.pause(0.05)
        return None

    def path_following_udfc(self, udfc_img):
        '''The function to rule them  all - this uses all the functions above'''

        #Killing the  node if it is not in its operating state(Given by the state machine-Subscribernode).
        if self.current_state not in self.possible_states:
            return None

        # Extracting the contour
        hog_img = self.findHOG(udfc_img)
        contour = self.threshold(hog_img)

        if self.isDetected:
            #Approximating the line
            alpha, beta = self.find_line(contour)

            if self.isDetected:
                #Estimating the next waypoint
                waypoint, q = self.estimate_next_waypoint(alpha, beta)

                pose_odom = self.map_to_odom(waypoint, q)

                #Publishing the next waypoint
                self.publish_waypoint(self.wpPub, self.objectID, pose_odom)

                # for visualization
                #self.plotting(contour, alpha, beta)

            else:
                rospy.loginfo('RANSAC failed')
                p = ObjectPosition()
                p.objectID = self.objectID
                p.isDetected = self.isDetected
                self.wpPub.publish(p)

                # for visualization
                self.plotting(contour)

        else:
            rospy.loginfo('Contour failed')
            p = ObjectPosition()
            p.objectID = self.objectID
            p.isDetected = self.isDetected
            self.wpPub.publish(p)

    def spin(self):
        """ running at a specific Hz """
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.path_following_udfc(self.udfc_img)
            rate.sleep()


if __name__ == '__main__':
    try:
        pipeline_following_node = PipelineFollowingNode()
        pipeline_following_node.spin()

    except rospy.ROSInterruptException:
        pass
