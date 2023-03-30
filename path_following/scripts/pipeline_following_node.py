#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
from vortex_msgs.msg import ObjectPosition
import numpy as np
from image_extraction import Image_extraction
from RANSAC import RANSAC, LinearRegressor
"""
Node made to publish data to the landmarkserver of type "Objectposition", which is an own defined Vortex msg and can be found in the vortex-msgs respository.
It takes in data from the UDFC (Monocamera facing downwards under Beluga). The "mission_topic_sub"-subscriber controls if our node is running.
It is running if the state = "pipeline/execute", "pipeline/standby".
For the TAC 2023, the execute state will be the most relevant, but we dont have to get as good data to estimate direction as they did in the 2022 Robosub.
This node is mainly a hard copy of the path_following_node, used at Robosub 2022, but with adaptions for the TAC2023"
Tuva  
"""


class PipelineFollowingNode():
    """
    Node for completing the pipeline following task in TAC 2023. By usage of feature 
    processing a contour is extracted befare a RANSAC algoritm is performed to extract
    a line. The parameters, alpha and beta, from the line is then used to calculate a 
    vector that estimates points that will guide the drone along the pipeline.
    """

    def __init__(self):
        rospy.init_node('pointcloud_processing_node')

        #Parameters for pipe color (lower-> more red, higher -> more green, yellow is around 60)
        self.lower_hue = 20
        self.upper_hue = 80
        #Parameters for waypoint estimation
        self.K1 = -0.05
        self.K2 = -0.5
        self.x_step = 0.5

        # Subscribers
        self.udfcSub = rospy.Subscriber("/cv/image_preprocessing/CLAHE/udfc",
                                        Image, self.path_following_udfc_cb)
        self.mission_topic_sub = rospy.Subscriber("/fsm/state", String,
                                                  self.update_mission)
        rospy.Subscriber("/odometry/filtered", Odometry, self.odom_cb)
        self.odom = Odometry

        #Topic to be subscribed by the landmark server
        self.wpPub = rospy.Publisher('/fsm/object_positions_in',
                                     ObjectPosition,
                                     queue_size=1)

        # Initialize state and bools
        self.possible_states = ["pipeline/execute", "pipeline/standby"]
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

        # Wait for first image
        self.bridge = CvBridge()
        img = rospy.wait_for_message(
            "/cv/image_preprocessing/CLAHE_single/udfc", Image)
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img, "passthrough")
            self.image_shape = cv_image.shape
            self.img_center = np.array(
                [self.image_shape[0] / 2, self.image_shape[1] / 2])

        except CvBridgeError:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def odom_cb(self, msg):
        self.odom = msg

    def publish_waypoint(self, publisher, objectID, waypoint, q):
        """
        Publishes a waypoint as an Objectposition-msg, using the given publisher. The interesting components are the Object Id, x and y positions and whether anything is detected or not.
        """

        #P is the object to be sent on the "/fsm/state"-topic, ObjectPosition is a Vortex-defined msg.
        p = ObjectPosition()
        #p.pose.header[]
        p.objectID = objectID

        p.objectPose.pose.position.x = waypoint[0]
        p.objectPose.pose.position.y = waypoint[1]
        p.objectPose.pose.position.z = waypoint[
            2]  #z is set to zero as default, we need to ignore depth data comming from this class.
        p.objectPose.pose.orientation.x = 0
        p.objectPose.pose.orientation.y = 0
        p.objectPose.pose.orientation.z = q[4]
        p.objectPose.pose.orientation.w = q[1]

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

        n = 10
        k = 1000
        t = 400
        d = np.size(points) / 4
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
        e1 = 620 - beta
        e2 = alpha
        theta = self.K1 * e1 + self.K2 * e2
        theta_rad = theta * 2 * np.pi / 360

        p0 = np.array([
            self.odom.pose.pose.position.x, self.odom.pose.pose.position.y,
            self.odom.pose.pose.position.z
        ])

        waypoint = p0 + self.x_step * np.array(
            [np.cos(theta_rad), np.sin(theta_rad), 0])
        q = np.array([np.cos(theta_rad / 2), 0, 0, np.sin(theta_rad / 2)])

        print('e1:', e1)
        print('e2:', e2)
        print('theta:', theta)
        print('Waypoint:', waypoint)

        return waypoint, q

    def findContour(self, img):

        contour = self.extractor.YellowEdgesHSV(img, self.lower_hue,
                                                self.upper_hue)
        points = np.argwhere(contour > 1)
        print('Number of points in contour: ', points[:, 0].size)

        if points[:, 0].size > self.detection_area_threshold:
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

        udfc_img = self.bridge.imgmsg_to_cv2(img_msg, "passthrough")

        # Extracting the contour
        contour = self.findContour(udfc_img)

        if contour is not None:
            #Approximating the line
            alpha, beta = self.find_line(contour)

            #Estimating the next waypoint
            waypoint, q = self.estimate_next_waypoint(alpha, beta)

            #Publishing the next waypoint
            self.publish_waypoint(self.wpPub, "Pipeline", waypoint, q)


if __name__ == '__main__':
    try:
        pipeline_following_node = PipelineFollowingNode()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
