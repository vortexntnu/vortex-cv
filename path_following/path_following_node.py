

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
        self.hsv_params = ["bruh is equal to 1"] # TODO: BenG help bruh not be equal to one. How does one HSV decide :(
        
        
        self.mission_topic_sub = rospy.Subscriber("/fsm/state", String, self.update_mission)
        self.feat_detSub = rospy.Subscriber('/feature_detection/object_points', PointArray, self.feat_det_cb)
        
        # List of states this algorithm runs in
        self.possible_states = ["path_search", "path_converge", "path_execute"]

        
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
    
    #def zed_feed_callback(self, img_msg):
    #    try:
    #        self.zed_image = self.bridge.imgmsg_to_cv2(img_msg, "passthrough")
    #    except CvBridgeError, e:
    #        rospy.logerr("CvBridge Error: {0}".format(e))
    #
#
    #def udfc_feed_callback(self, img_msg):
    #    try:
    #        self.udfc_image = self.bridge.imgmsg_to_cv2(img_msg, "passthrough")
    #    except CvBridgeError, e:
    #        rospy.logerr("CvBridge Error: {0}".format(e))


    def path_following_zed_cb(self, img_msg):
        """
        We only use data from the ZED if we are searching for the path

        Find the biggest contour in the image and return its range and bearing
        Instead of range and bearing, we might want to actually do a 3D point and just say we are unsure of the distance

        """
        if self.current_state != "path_search":
            return None

        zed_image = self.bridge.imgmsg_to_cv2(img_msg, "passthrough")
        

    def path_following_udfc_cb(self, img_msg):

        """
        The first time we classify the path in UDFC makes us move to converge state

        Simplest Way to do this: Find biggest contour and if it is above some threshold for area declare the path there

        """

        if self.current_state not in self.possible_states:
            return None

        udfc_img = self.bridge.imgmsg_to_cv2(img_msg, "passthrough")
        udfc_img_r = udfc_img[:,:,2]

        # Apply HSV to image
        _, hsv_mask, hsv_mask_validation_img = self.udfc_feature_detector.hsv_processor(udfc_img, 
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

        cv2.drawContours(udfc_img, path_contour, (0,255,0), 5)
        img_with_circle = cv2.circle(udfc_img, (cx,cy), radius=1, color=(0, 0, 255), thickness=-1)

        

        
        # For testing
        
    
    # My feeling right now (kl 14:05 on sunday): We should not do the spiny stuff but do individual callbacks when we get camera msgs
    def spin(self):
        while not rospy.is_shutdown():            
            if self.current_state == "path_search" and self.zed_image is not None:
                # Do ZED stuff
                # TODO: figure out how to do this


                # Calculations
                bbox_points, bbox_area, points_in_rects, detection = self.feat_detection.classification(self.cv_image, self.current_object, self.hsv_params, self.noise_rm_params)
                pt_arr_msg = self.build_point_array_msg(points_in_rects, self.current_object, self.image_shape[0], self.image_shape[1])
                self.RectPointsPub.publish(pt_arr_msg)

                # Publishing
                self.cv_image_publisher(self.hsvCheckPub, self.feat_detection.hsv_validation_img)
                self.cv_image_publisher(self.noiseRmPub, self.feat_detection.nr_img, msg_encoding="mono8")
                self.cv_image_publisher(self.i2rcpPub, self.feat_detection.i2rcp_image_blank)
                self.cv_image_publisher(self.shapePub, self.feat_detection.rect_flt_img)
                self.cv_image_publisher(self.linesPub, self.feat_detection.line_fitting_img)
                self.cv_image_publisher(self.BBoxPub, self.feat_detection.bbox_img)
                self.cv_image_publisher(self.pointAreasPub, self.feat_detection.pointed_rects_img)

                    if bbox_points:
                        bboxes_msg = self.build_bounding_boxes_msg(bbox_points, self.current_object)
                        self.BBoxPointsPub.publish(bboxes_msg)
                    
                        self.prev_bboxes_msg = bboxes_msg
                    
                    else:
                        rospy.logwarn("Bounding Box wasnt found... keep on spinning...")
                        self.BBoxPointsPub.publish(self.prev_bboxes_msg)
                    
                    end = timer() # Stop function timer.
                    timediff = (end - start)
                    fps = 1 / timediff # Take reciprocal of the timediff to get runs per second. 
                    self.timerPub.publish(fps)
                

            if self.current_state in self.possible_states and self.udfc_image is not None:
                # Do UDFC stuff

            self.ros_rate.sleep()
        



if __name__ == '__main__':
    try:
        feature_detection_node = PathFollowingNode(image_topic='/zed2/zed_node/rgb/image_rect_color')
        # rospy.spin()
        feature_detection_node.spin()

    except rospy.ROSInterruptException:
        pass