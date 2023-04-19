#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
from vortex_msgs.msg import ObjectPosition
import numpy as np
from image_extraction import Image_extraction
from RANSAC import RANSAC
from HOG import HOG
from matplotlib import pyplot as plt
import tf2_ros
import geometry_msgs.msg
import tf2_geometry_msgs
import cv2 as cv
"""
Node made to publish data to the landmarkserver of type "Objectposition", which is an own defined Vortex msg and can be found in the vortex-msgs respository.
It takes in data from the UDFC (Monocamera facing downwards under Beluga). The "mission_topic_sub"-subscriber controls if our node is running.
It is running if the state = "pipeline/execute", "pipeline/standby".
For the TAC 2023, the execute state will be the most relevant, but we dont have to get as good data to estimate direction as they did in the 2022 Robosub.
This node is mainly a hard copy of the path_following_node, used at Robosub 2022, but with adaptions for the TAC2023"
Tuva and Lasse  
"""


class PipelineFollowingNode():
    """
    Node for completing the pipeline following task in TAC 2023. By usage of feature 
    processing a contour is extracted befare a RANSAC algoritm is performed to extract
    a line. The parameters, alpha and beta, from the line is then used to calculate a 
    vector that estimates points that will guide the drone along the pipeline.
    """

    def __init__(self):
        rospy.init_node('pipeline_following_node')

        ### Tuning ###
        #Parameters for pipe color (lower-> more red, higher -> more green, yellow is around 60)
        self.lower_hue = 5
        self.upper_hue = 170
        #Parameters for waypoint estimation
        self.K1 = -0.08  # beta error
        self.K2 = -10  # alpha error
        self.x_step = 0.2  # meters ahead of drone
        #Parameters RANSAC
        self.n = 10  # `n`: Minimum number of data points to estimate parameters
        self.k = 100  # `k`: Maximum iterations allowed
        self.t = 20  # `t`: Threshold value to determine if points are fit well
        self.d = None  # `d`: Number of close data points required to assert model fits well

        # Parameters
        self.count = 0
        self.img_size = []

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
        self.dataPub = rospy.Publisher('/visualize_contour',
                                       Image,
                                       queue_size=1)

        # Initialize state and bools
        self.possible_states = ["pipeline/execute", "pipeline/standby"]
        self.current_state = ""
        self.objectID = "pipeline"
        self.detection_area_threshold = 10

        self.isDetected = False
        self.estimateConverged = False
        self.estimateFucked = False

        self.extractor = Image_extraction()

        # Wait for first image
        self.bridge = CvBridge()
        print('wait for picture')
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
        self.count += 1

    def map_to_odom(self, p_baselink, q_baselink):
        """
        Takes a pose in base_link coordinates and calculates it in odom frame.

        Input: 
            p_baselink - point in base link
            q_baselink - quaternions in base link

        Output:
            pose_odom - estimate of point/pose in odom frame
        """

        pose_base_link = geometry_msgs.msg.PoseStamped()
        pose_base_link.header.frame_id = self.child_frame
        pose_base_link.header.stamp = rospy.Time.now()
        pose_base_link.pose.position.x = p_baselink[0]
        pose_base_link.pose.position.y = p_baselink[1]
        pose_base_link.pose.position.z = p_baselink[2]

        pose_base_link.pose.orientation.x = q_baselink[3]
        pose_base_link.pose.orientation.y = q_baselink[2]
        pose_base_link.pose.orientation.z = q_baselink[3]
        pose_base_link.pose.orientation.w = q_baselink[0]

        transform = self.__tfBuffer.lookup_transform(self.parent_frame,
                                                     self.child_frame,
                                                     rospy.Time(),
                                                     rospy.Duration(5))

        pose_odom = tf2_geometry_msgs.do_transform_pose(
            pose_base_link, transform)

        return pose_odom

    def publish_waypoint(self, publisher, objectID, pose):
        """
        Publishes a waypoint as an Objectposition-msg, using the given publisher. 
        The interesting components are the Object Id, x and y positions and whether 
        anything is detected or not.
        """

        #P is the object to be sent on the "/fsm/state"-topic, ObjectPosition is a Vortex-defined msg.
        p = ObjectPosition()
        #p.pose.header[]
        p.objectID = objectID

        p.objectPose.pose = pose.pose

        p.isDetected = self.isDetected
        p.estimateConverged = self.estimateConverged
        p.estimateFucked = self.estimateFucked

        #Function-call, takes in a publisher as a variable.
        publisher.publish(p)

        rospy.loginfo("Object published: %s", objectID)

    def find_line(self, contour):
        """
        Uses RANSAC to find line and returns 
        line parameters alpha and beta
        """
        points = np.argwhere(contour > 0)
        X = points[:, 0].reshape(-1, 1)
        y = points[:, 1].reshape(-1, 1)

        self.d = np.size(points) / 8
        regressor = RANSAC(self.n, self.k, self.t, self.d)

        regressor.fit(X, y)
        if regressor.fail:
            self.isDetected = False
            return None, None

        params = regressor.best_fit.params
        alpha = float(params[1])
        beta = float(params[0])

        print('points:', regressor.points.size)

        return alpha, beta

    def estimate_next_waypoint(self, alpha, beta):
        """
        Not ftested in any way!
        Brainstorming:
        -Assuming we have line fromfind line-function.
        -Needs to be found from current position
        -Cameraframe: line
        -Odomframe: current position, output position
        -Output:  waypoint, 3 state array, z = 0
        """

        #Error
        e1 = self.image_shape[1] / 2 - beta
        e2 = alpha
        theta = self.K1 * e1 + self.K2 * e2
        if theta > 90:
            theta = 90
        if theta < -90:
            theta = -90
        theta_rad = theta * 2 * np.pi / 360

        waypoint = self.x_step * np.array(
            [np.cos(theta_rad), np.sin(theta_rad), 0])
        q = np.array([np.cos(theta_rad / 2), 0, 0, np.sin(theta_rad / 2)])

        print('alpha:', alpha)
        print('beta', beta)
        print('e1:', e1)
        print('e2:', e2)
        print('theta:', theta)
        print('Waypoint:', waypoint)

        return waypoint, q

    def findContour(self, img):
        extractor = HOG(img)
        features, contour = extractor.compute_hog()
        #contour = self.extractor.YellowEdgesHSV(img, self.lower_hue,
        #                                       self.upper_hue)
        threshold = 0.5 * contour.max()
        if threshold < 2:
            threshold = 2
        binary_image = contour > threshold
        points = np.argwhere(contour > 0)
        print('Number of points in contour: ', points[:, 0].size)

        if points[:, 0].size > self.detection_area_threshold:
            self.isDetected = True
            return binary_image
        else:
            self.isDetected = False
            return None

    #The function to rule them  all
    def path_following_udfc(self, udfc_img, count):

        #Killing the  node if it is not in its operating state(Given by the state machine-Subscribernode).
        # if self.current_state not in self.possible_states:
        #     return None

        # Extracting the contour
        contour = self.findContour(udfc_img)
        data = Image()
        data.data = contour
        self.dataPub.publish(data)
        print(self.image_shape)

        if contour is not None:
            #Approximating the line
            alpha, beta = self.find_line(contour)

            if self.isDetected:
                #Estimating the next waypoint
                waypoint, q = self.estimate_next_waypoint(alpha, beta)

                pose_odom = self.map_to_odom(waypoint, q)

                #Publishing the next waypoint
                self.publish_waypoint(self.wpPub, self.objectID, pose_odom)
                print('isDetected:', self.isDetected)

                #Plotting
                points = np.argwhere(contour > 0)
                print(points[:, 0].size)
                plt.figure(1, figsize=(10, 14))
                plt.clf()
                plt.axes().set_aspect('equal')
                plt.xlim(0, 588)
                plt.ylim(0, 795)
                plt.scatter(points[:, 0], points[:, 1])
                x = np.linspace(0, 1239, num=100).reshape(-1, 1)
                plt.plot(x, alpha * x + beta)
                plt.pause(0.05)
            else:
                cv.imwrite('ransac_fail_image.jpg', udfc_img)
                print('Image number:', count)
                p = ObjectPosition()
                p.objectID = self.objectID
                p.isDetected = self.isDetected
                print('isDetected:', self.isDetected)
                self.wpPub.publish(p)
                print('RANSAC failed')

                points = np.argwhere(contour > 0)
                print(points[:, 0].size)
                plt.figure(1, figsize=(10, 10))
                plt.clf()
                plt.axes().set_aspect('equal')
                plt.xlim(0, 588)
                plt.ylim(0, 795)
                plt.scatter(points[:, 0], points[:, 1])
                plt.pause(0.05)

        else:
            print("The data type of the image is", udfc_img.dtype)
            print('image shape', udfc_img.shape)
            cv.imwrite('contour_fail_image.jpg', udfc_img)
            print('Image number:', count)
            p = ObjectPosition()
            p.objectID = self.objectID
            p.isDetected = self.isDetected
            print('isDetected:', self.isDetected)
            print('Contour failed')
            self.wpPub.publish(p)

    def spin(self):
        """ running at a specific Hz """
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.path_following_udfc(self.udfc_img, self.count)
            rate.sleep()


if __name__ == '__main__':
    try:
        pipeline_following_node = PipelineFollowingNode()
        pipeline_following_node.spin()

    except rospy.ROSInterruptException:
        pass
