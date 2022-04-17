

## TODO: imports taken from feature detection, remove what is unused

import rospy
import rospkg

from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Empty, String
from cv_msgs.msg import Point2, PointArray
from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes

from cv_bridge import CvBridge, CvBridgeError
import dynamic_reconfigure.client

import numpy as np
from timeit import default_timer as timer
import traceback
import cv2

import scripts.feature_detection

class PathFollowingNode():
    """
    Node for comperhensive solution to the path following task in RoboSub 2022 on Perception side
    The Node only operates when fsm is in path_search/converge/execute state, and governs the transition between these
    
    3 main problems need to be solved by the node, and they are associated to the possible states for path:

        path_search:        During path search the node uses ZED camera feed to detect the path. After path is detected, the coordinates are published to Autonomous
                            and we request switch to converge.

        path_converge:      During path converge the goal is to center the path in the FOV of the underwater downwards facing camera (in the future refered to as the UDFC)
                            The path is classified in UDFC frame and it's areal centroid is mapped to odom and given as xy reference for the DP controller. The assumption being
                            that this scheme will progressively get the path more and more into FOV, and finally in the centre of UDFC frame. After the path is sufficiently
                            centered we move on to execute.

        path_execute:       During execute the drone keeps station over the path while Perception solves the estimation problem of the path to the next task. Here we can use
                            the fact that we are given the oportunity for multiple measurements and samples to cast the task as a batch problem and extract as much information
                            as possible before we declare a solution. The search for the next task starts when a waypoint is given to Autonomous after we have estimated path 
                            direction.
    """

    def __init__(self):
        rospy.init_node('pointcloud_processing_node')
        
        # Parameters for the algorithm:
        # TODO: make these dynamic parameters
        self.hsv_params = [0,
                           179,
                           0,
                           255,
                           0,
                           255]
        
        self.zedSub = rospy.Subscriber("/cv/image_preprocessing/CLAHE_single/zed2", Image, self.path_following_zed_cb)
        self.udfcSub = rospy.Subscriber("/cv/image_preprocessing/CLAHE_single/udfc", Image, self.path_following_zed_cb)
        self.mission_topic_sub = rospy.Subscriber("/fsm/state", String, self.update_mission)

        self.zedPub = rospy.Publisher("/path_following/zed2", Image, queue_size=1)
        self.udfcPub = rospy.Publisher("/path_following/udfc", Image, queue_size=1)
    
        self.bridge = CvBridge()


        # List of states this algorithm runs in
        self.possible_states = ["path_search", "path_converge", "path_execute"]

        # TODO: make a subscriber for this
        self.current_state = "path_search"
        
        # TODO: get image sizes for both ZED and UDFC cameras, these will be used to make the fd objects
        # First initialization of image shape
        first_image_msg = rospy.wait_for_message(image_topic, Image)
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(first_image_msg, "passthrough")
            self.udfc_image_shape = self.cv_image.shape
        except CvBridgeError, e:
            self.image_shape = (720, 1280, 4)


        # Defining classes
        self.udfc_feature_detector = scripts.feature_detection.FeatureDetection()
    
    def fsm_state_callback(self, fsm_msg):
        # TODO: fix this for current state
        self.current_state = 1

    def odom_tf_callback(self, tf_msg):
        # TODO: fix this for transformations
        self.last_odom = 1
    

    def path_contour_and_centroid(self, img):
        """
        Takes in an image and finds the path contour and centroid. Also returns the original image with these drawn.

        """
        #img_r = img

        # Apply HSV to image
        _, hsv_mask, _ = self.udfc_feature_detector.hsv_processor(img, 
                                        self.hsv_params[0], self.hsv_params[1], self.hsv_params[2],
                                        self.hsv_params[3], self.hsv_params[4], self.hsv_params[5])

        # Filter the image for noise
        noise_filtered_img = self.udfc_feature_detector.noise_removal_processor(hsv_mask)

        # Get most probable path contour
        path_contour = self.udfc_feature_detector.contour_processing(noise_filtered_img, contour_area_threshold=3000, 
                                                                return_image=False)

        path_contour = self.udfc_feature_detector.contour_processing(noise_filtered_img, contour_area_threshold=3000, variance_filtering=True, coloured_img=udfc_img_r)

        M = cv2.moments(path_contour)

        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])

        cv2.drawContours(img, path_contour, (0,0,255), 5)
        img_drawn = cv2.circle(img, (cx,cy), radius=1, color=(0, 0, 255), thickness=-1)

        return path_contour, [cx, cy], img_drawn


    def cv_image_publisher(self, publisher, image, msg_encoding="bgra8"):
        """
        Takes a cv::Mat image object, converts it into a ROS Image message type, and publishes it using the specified publisher.
        """
        msgified_img = self.bridge.cv2_to_imgmsg(image, encoding=msg_encoding)
        publisher.publish(msgified_img)



    def path_following_zed_cb(self, img_msg):
        """
        We only use data from the ZED if we are searching for the path

        Find the biggest contour in the image and return its range and bearing
        Instead of range and bearing, we might want to actually do a 3D point and just say we are unsure of the distance

        """
        if self.current_state != "path_search":
            return None

        zed_img = self.bridge.imgmsg_to_cv2(img_msg, "passthrough")

        path_contour, path_centroid, img_drawn = self.path_contour_and_centroid(zed_img)

        self.cv_image_publisher(self.zedPub, img_drawn)
        

    def path_following_udfc_cb(self, img_msg):

        """
        The first time we classify the path in UDFC makes us move to converge state

        Simplest Way to do this: Find biggest contour and if it is above some threshold for area declare the path there

        """

        if self.current_state not in self.possible_states:
            return None

        udfc_img = self.bridge.imgmsg_to_cv2(img_msg, "passthrough")
        
        path_contour, path_centroid, img_drawn = self.path_contour_and_centroid(udfc_img)

        self.cv_image_publisher(self.udfcPub, img_drawn)


if __name__ == '__main__':
    try:
        path_following_node = PathFollowingNode()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass