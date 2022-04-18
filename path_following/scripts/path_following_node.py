#!/usr/bin/env python

## TODO: imports taken from feature detection, remove what is unused

import rospy
import rospkg

from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Empty, String
from cv_msgs.msg import Point2, PointArray
#from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes

from cv_bridge import CvBridge, CvBridgeError
import dynamic_reconfigure.client

import numpy as np
from timeit import default_timer as timer
import traceback
import cv2

import feature_detection

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
        
        self.hsv_params = [0,
                           179,
                           0,
                           255,
                           0,
                           255]
        
        
        self.ksize1 = 7
        self.ksize2 = 7
        self.sigma = 0.8

        # Thresholding params
        self.thresholding_blocksize = 11
        self.thresholding_C = 2

        # Erosion and dilation params
        self.erosion_dilation_ksize = 5
        self.erosion_iterations = 1
        self.dilation_iterations = 1
        self.noise_rm_params = [self.ksize1, self.ksize2, self.sigma, self.thresholding_blocksize, self.thresholding_C, self.erosion_dilation_ksize, self.erosion_iterations, self.dilation_iterations]
        
        #self.zedSub = rospy.Subscriber("/cv/image_preprocessing/CLAHE/zed2", Image, self.path_following_zed_cb)
        self.udfcSub = rospy.Subscriber("/cv/image_preprocessing/CLAHE/udfc", Image, self.path_following_udfc_cb)
        #self.mission_topic_sub = rospy.Subscriber("/fsm/state", String, self.update_mission)

        self.zedPub = rospy.Publisher("/path_following/zed2", Image, queue_size=1)
        self.udfcPub = rospy.Publisher("/path_following/udfc", Image, queue_size=1)
        
        self.hsvPub = rospy.Publisher("/path_following/hsv_monkey", Image, queue_size=1)
        
        
        self.noise_filteredPub = rospy.Publisher("/path_following/noise_filtered", Image, queue_size=1)
        self.bridge = CvBridge()

        # TODO: 1. Solve the udfc and zed cb problems, probably make multiple callbacks even if BenG will hate me.
        #       2. Extract a 3D point from Zed topic to publish, and extract a 2D point from UDFC to publish.
        #       3. Get some filtering action going for that centroid

        # List of states this algorithm runs in
        self.possible_states = ["path_search", "path_converge", "path_execute"]

        # TODO: make a subscriber for this
        self.current_state = "path_search"
        
        # TODO: get image sizes for both ZED and UDFC cameras, these will be used to make the fd objects
        # First initialization of image shape

        # Defining classes
        img = rospy.wait_for_message("/cv/image_preprocessing/CLAHE_single/udfc", Image)
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img, "passthrough")
        except CvBridgeError, e:
            rospy.logerr("CvBridge Error: {0}".format(e))

        self.feature_detector = feature_detection.FeatureDetection(cv_image.shape)
        
        self.dynam_client = dynamic_reconfigure.client.Client("/CVOD_cfg/feature_detection_cfg", config_callback=self.dynam_reconfigure_callback)
    
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
        
        # Apply HSV to image
        _, hsv_mask, hsv_val_img = self.feature_detector.hsv_processor(img, *self.hsv_params)
        self.cv_image_publisher(self.hsvPub, hsv_val_img, msg_encoding="bgr8")

        

        # Filter the image for noise
        noise_filtered_img = self.feature_detector.noise_removal_processor(hsv_mask, *self.noise_rm_params)
        self.cv_image_publisher(self.noise_filteredPub, noise_filtered_img, msg_encoding="8UC1")

        # Get most probable path contour
        #path_contour = self.feature_detector.contour_processing(noise_filtered_img, contour_area_threshold=3000, return_image=False)

        path_contour = self.feature_detector.contour_processing(noise_filtered_img, contour_area_threshold=3000, variance_filtering=True, coloured_img=img, return_image=False)
        #rospy.logwarn(path_contour)
        
        cv2.drawContours(img, path_contour, -1, (0,0,255), 5)
        
        M = cv2.moments(path_contour[0])
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        img_drawn = cv2.circle(img, (cx,cy), radius=1, color=(0, 255, 0), thickness=3)

        return path_contour, [cx, cy], img_drawn, hsv_val_img


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
        
        path_contour, path_centroid, img_drawn, hsv_val_img = self.path_contour_and_centroid(zed_img)

        self.cv_image_publisher(self.zedPub, img_drawn)
        

    def path_following_udfc_cb(self, img_msg):

        """
        The first time we classify the path in UDFC makes us move to converge state

        Simplest Way to do this: Find biggest contour and if it is above some threshold for area declare the path there

        """

        if self.current_state not in self.possible_states:
            return None

        udfc_img = self.bridge.imgmsg_to_cv2(img_msg, "passthrough")
        
        path_contour, path_centroid, img_drawn, hsv_val_img = self.path_contour_and_centroid(udfc_img)
        
        self.cv_image_publisher(self.udfcPub, img_drawn, "bgr8")



    def dynam_reconfigure_callback(self, config):
        
        
        self.canny_threshold1 = config.canny_threshold1
        self.canny_threshold2 = config.canny_threshold2
        self.canny_aperture = config.canny_aperture_size

        self.hsv_params[0] = config.hsv_hue_min
        self.hsv_params[1] = config.hsv_hue_max
        self.hsv_params[2] = config.hsv_sat_min
        self.hsv_params[3] = config.hsv_sat_max
        self.hsv_params[4] = config.hsv_val_min
        self.hsv_params[5] = config.hsv_val_max

        self.ksize1 = config.ksize1
        self.ksize2 = config.ksize2
        self.sigma = config.sigma

        self.thresholding_blocksize = config.blocksize
        self.thresholding_C = config.C

        self.erosion_dilation_ksize = config.ed_ksize
        self.erosion_iterations = config.erosion_iterations
        self.dilation_iterations = config.dilation_iterations

        self.noise_rm_params = [self.ksize1, self.ksize2, self.sigma, self.thresholding_blocksize, self.thresholding_C, self.erosion_dilation_ksize, self.erosion_iterations, self.dilation_iterations]
        
if __name__ == '__main__':
    try:
        path_following_node = PathFollowingNode()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass