#!/usr/bin/env python

## TODO: imports taken from feature detection, remove what is unused

from matplotlib.pyplot import contour
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
from geometry_msgs.msg import TransformStamped

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

        # For publishing results for Auto
        self.parent_frame = 'odom' 
        self.child_frame = 'udfc_link'
        
        # Parameters for the algorithm:
        
        self.hsv_params = [0,       #hsv_hue_min
                           53,     #hsv_hue_max
                           71,       #hsv_sat_min
                           131,     #hsv_sat_max
                           175,       #hsv_val_min
                           248]     #hsv_val_max
        
      
        self.ksize1 = 7
        self.ksize2 = 7
        self.sigma = 0.8

        self.batch_line_params = np.zeros((1,4))
        # Thresholding params
        self.thresholding_blocksize = 11
        self.thresholding_C = 2

        # Erosion and dilation params
        self.erosion_dilation_ksize = 5
        self.erosion_iterations = 1
        self.dilation_iterations = 1
        self.noise_rm_params = [self.ksize1, self.ksize2, self.sigma, self.thresholding_blocksize, self.thresholding_C, self.erosion_dilation_ksize, self.erosion_iterations, self.dilation_iterations]
        
        self.udfcSub = rospy.Subscriber("/cv/image_preprocessing/CLAHE/udfc", Image, self.path_following_udfc_cb)
        #self.mission_topic_sub = rospy.Subscriber("/fsm/state", String, self.update_mission)

        self.zedPub = rospy.Publisher("/path_following/zed2", Image, queue_size=1)
        self.udfcPub = rospy.Publisher("/path_following/udfc", Image, queue_size=1)
        
        self.hsvPub = rospy.Publisher("/path_following/hsv_monkey", Image, queue_size=1)
        
        # Publish to autonomous
        self.pathPub = rospy.Publisher('/fsm/object_positions_in', ObjectPosition, queue_size=1)
        #self.pointPub = rospy.Publisher('/path_following/path_centriod', PointStamped, queue_size= 1)

        self.noise_filteredPub = rospy.Publisher("/path_following/noise_filtered", Image, queue_size=1)
        self.bridge = CvBridge()

        # TODO: 1. Solve the udfc and zed cb problems, probably make multiple callbacks even if BenG will hate me.
        #       2. Extract a 3D point from Zed topic to publish, and extract a 2D point from UDFC to publish.
        #       3. Get some filtering action going for that centroid

        # List of states this algorithm runs in
        self.possible_states = ["path_search", "path_converge", "path_execute"]

        # TODO: make a subscriber for this
        self.current_state = "path_search"
        self.objectID = "path"
        self.isDetected = False
        self.detection_area_threshold = 2000
        # TODO: get image sizes for both ZED and UDFC cameras, these will be used to make the fd objects
        # First initialization of image shape

        # TODO: In case transform performs badly, use this to estimate 
        #cv2.calibrationMatrixValues

        # Focal length 2.97 mm from the camera specs
        self.focal_length = 2.97 * 10 **-3

        # TODO: find out what this is for optimal PFPSPerformance
        self.Z_prior = 1.69

        # TODO: Henrique pliz gibe:
        self.camera_matrix = np.eye(3)
        self.udfc_dcs = [1, 2, 3, 4, 5, 6]

        # Defining classes
        img = rospy.wait_for_message("/cv/image_preprocessing/CLAHE_single/udfc", Image)
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img, "passthrough")
        except CvBridgeError, e:
            rospy.logerr("CvBridge Error: {0}".format(e))

        self.feature_detector = feature_detection.FeatureDetection(cv_image.shape)
        
        self.dynam_client = dynamic_reconfigure.client.Client("/CVOD_cfg/feature_detection_cfg", config_callback=self.dynam_reconfigure_callback)
        rospy.loginfo("Bruh do we get here????")
        #TF stuff
        #self.__tfBuffer = tf2_ros.Buffer()
        #self.__listener = tf2_ros.TransformListener(self.__tfBuffer)
        #self.__tfBroadcaster = tf2_ros.TransformBroadcaster()
#
        ##The init will only continue if a transform between parent frame and child frame can be found
        #while self.__tfBuffer.can_transform(self.parent_frame, self.child_frame, rospy.Time()) == 0:
        #    try:
        #        rospy.loginfo("No transform between "+str(self.parent_frame) +' and ' + str(self.child_frame))
        #        rospy.sleep(2)
        #    except: #, tf2_ros.ExtrapolationException  (tf2_ros.LookupException, tf2_ros.ConnectivityException)
        #        rospy.sleep(2)
        #        continue
        #
        #rospy.loginfo("Transform between "+str(self.parent_frame) +' and ' + str(self.child_frame) + 'found.')
    
    #def fsm_state_callback(self, fsm_msg):
    #    # TODO: fix this for current state
    #    self.current_state = 1
#
    #def odom_tf_callback(self, tf_msg):
    #    # TODO: fix this for transformations
    #    self.last_odom = 1
#
    #def make_pointStamped_odom(self, headerdata, position, name):
    #    # TODO: someone else fix the fucking tf stuff I hate this fucking piece of shit
#
    #    new_point = PointStamped()
    #    new_point.header = "path_centroid"
    #    new_point.header.stamp = rospy.get_rostime()
    #    new_point.point.x = position[0]
    #    new_point.point.y = position[1]
    #    new_point.point.z = position[2]
    #    pointPub.publish(new_point)
        rospy.loginfo("Bruh do we get here????")

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
    

    def publish_object(self, path, path_centroid):
        p = ObjectPosition()
        #p.pose.header[]
        p.objectID = self.objectID
        p.objectPose.header = "object_" + str(self.objectID)
        p.objectPose.pose.position.x = path_centroid[0]
        p.objectPose.pose.position.y = path_centroid[1]
        p.objectPose.pose.position.z = path_centroid[2]
        p.objectPose.pose.orientation.x = 0
        p.objectPose.pose.orientation.y = 0
        p.objectPose.pose.orientation.z = 0
        p.objectPose.pose.orientation.w = 0
        
        # TODO: figure out bools with auto

        self.pathPub(p)
        self.transformbroadcast(self.parent_frame, p)

    def path_calculations(self, img):
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
        #rospy.logwarn(path_contour[0])
        
        cv2.drawContours(img, path_contour, -1, (0,0,255), 5)
        path_area = cv2.contourArea(path_contour[0])
        
        # This is just for the purposes of visualization on the image
        M = cv2.moments(path_contour[0])
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        img_drawn = cv2.circle(img, (cx,cy), radius=1, color=(0, 255, 0), thickness=3)

        return path_contour, path_area, img_drawn, hsv_val_img


    def cv_image_publisher(self, publisher, image, msg_encoding="bgra8"):
        """
        Takes a cv::Mat image object, converts it into a ROS Image message type, and publishes it using the specified publisher.
        """
        msgified_img = self.bridge.cv2_to_imgmsg(image, encoding=msg_encoding)
        publisher.publish(msgified_img)
        

    def find_contour_line(self, contour, img_drawn):
        vx, vy, x0, y0 = cv2.fitLine(contour, cv2.DIST_L2, 0, 0.1, 0.1)

        colin_vec = np.ravel(np.array((vx, vy)))
        p0 = np.ravel(np.array((x0, y0)))

        p1 = p0 + 1000*colin_vec
        p2 = p0 - 1000*colin_vec
        
        print(p1)
        print(p2)

        cv2.line(img_drawn, tuple(p1), tuple(p2), color=(0, 255, 0), thickness=2)

        return [vx, vy, x0, y0], img_drawn

    def path_following_udfc_cb(self, img_msg):

        """
        The first time we classify the path in UDFC makes us move to converge state

        Simplest Way to do this: Find biggest contour and if it is above some threshold for area declare the path there

        """

        if self.current_state not in self.possible_states:
            return None

        if np.shape(self.batch_line_params)[0] > 100:
            # TODO: write batch_estimate_waypoint and the waypoint publisher
            self.isEstimated = True
            next_waypoint = self.batch_estimate_waypoint()
            self.publish_waypoint(self.waypointPub, next_waypoint)
            
            return None

        udfc_img = self.bridge.imgmsg_to_cv2(img_msg, "passthrough")
        
        self.path_contour, path_area, img_drawn, hsv_val_img = self.path_calculations(udfc_img)
        self.path_contour = self.path_contour[0][:,0,:]
        
        if self.isDetected == False and path_area > self.detection_area_threshold:
            self.isDetected == True
        
        # TODO: test if this gives same result as first finding the centroid in image and them mapping it
        #path_contour_camera = cv2.undistortPoints(path_contour, cameraMatrix=self.camera_matrix, dts=self.udfc_dcs)

        M = cv2.moments(self.path_contour)
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        
        self.path_centroid = np.array(cx, cy)
        
        # TODO: Transform from image coords to camera coords
        cx_tilde, cy_tilde = cv2.unrectify(bruh = 1)
        
        X = (self.Z_prior / self.focal_length) * cx_tilde
        Y = (self.Z_prior / self.focal_length) * cy_tilde
        dp_ref = [X, Y, self.Z_prior]
        
        #print(np.shape(np.ravel(path_contour[0])))
        upper_inds = np.where((self.path_contour[:,1] < cy_tilde) == True)[0]
        upper_contour = self.path_contour[upper_inds]


        line_params, img_drawn = self.find_contour_line(upper_contour, img_drawn)

        self.batch_line_params = np.hstack((self.batch_line_params, line_params))

        #self.dp_ref_publisher(self.dp_ref_Pub, dp_ref)

        #if self.current_state == "path_search" or "path_converge":
        #    # TODO publish centroid here
        #    bruh = 1
#
        #elif self.current_state == "path_execute":
        #    bruh = 1
        #else:
        #    pass
        ## TODO: do the decision for straight vs bendy and the batch optimization to solve for path angle
        self.cv_image_publisher(self.udfcPub, img_drawn, "bgr8")
        self.publish_waypoint(self.waypointPub, next_waypoint)


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