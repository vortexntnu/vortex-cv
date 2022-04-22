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

import tf2_ros
import tf2_geometry_msgs.tf2_geometry_msgs

import numpy as np
from timeit import default_timer as timer
import traceback
import cv2
from os.path import join

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
        
        # Camera Calibration Params (CCP)
        param_folder = '../params'

        self.K           = np.loadtxt(join(param_folder, 'K.txt'))
        self.dcs         = np.loadtxt(join(param_folder, 'dc.txt'))
        self.K_opt       = np.loadtxt(join(param_folder, 'K_opt.txt'))

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
        
        # Thresholding params
        self.thresholding_blocksize     = 11
        self.thresholding_C             = 2

        # Erosion and dilation params
        self.erosion_dilation_ksize     = 5
        self.erosion_iterations         = 1
        self.dilation_iterations        = 1
        self.noise_rm_params = [self.ksize1, self.ksize2, self.sigma, self.thresholding_blocksize, self.thresholding_C, self.erosion_dilation_ksize, self.erosion_iterations, self.dilation_iterations]
        

        # Subscribers
        self.udfcSub = rospy.Subscriber("/cv/image_preprocessing/CLAHE/udfc", Image, self.path_following_udfc_cb)
        self.mission_topic_sub = rospy.Subscriber("/fsm/state", String, self.update_mission)

        # Publishers
        self.udfcPub = rospy.Publisher("/path_following/udfc", Image, queue_size=1)
        self.hsvPub = rospy.Publisher("/path_following/hsv_monkey", Image, queue_size=1)
        self.wpPub = rospy.Publisher('/fsm/object_positions_in', ObjectPosition, queue_size=1)

        self.noise_filteredPub = rospy.Publisher("/path_following/noise_filtered", Image, queue_size=1)
        self.bridge = CvBridge()


        # Initialize state and bools
        self.possible_states = ["path_search", "path_converge", "path_execute"]
        self.current_state = ""
        self.objectID = "path"
        self.detection_area_threshold = 2000

        self.isDetected            = False
        self.estimateConverged     = False
        self.estimateFucked        = False

        self.batch_line_params = np.zeros((1,4))

        # TODO: In case transform performs badly due to focal length, use this to estimate focal length
        #cv2.calibrationMatrixValues

        # Focal length 2.97 mm from the camera specs
        self.focal_length = 2.97 * 10 **-3

        # TODO: find realistic values for these
        self.H_pool_prior       = 2     # Depth of pool
        self.h_path_prior       = 0.1   # Height of path
        self.Z_prior_ref        = 1.69  # Optimal z-coordinate for performing pfps

        self.point_transformer = tf2_geometry_msgs.tf2_geometry_msgs

        # Wait for first image
        img = rospy.wait_for_message("/cv/image_preprocessing/CLAHE_single/udfc", Image)
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img, "passthrough")
        except CvBridgeError, e:
            rospy.logerr("CvBridge Error: {0}".format(e))

        # Objects for the classes
        self.feature_detector = feature_detection.FeatureDetection(cv_image.shape)
        self.dynam_client = dynamic_reconfigure.client.Client("/CVOD_cfg/feature_detection_cfg", config_callback=self.dynam_reconfigure_callback)

        #TF stuff
        self.__tfBuffer = tf2_ros.Buffer()
        self.__listener = tf2_ros.TransformListener(self.__tfBuffer)
        self.__tfBroadcaster = tf2_ros.TransformBroadcaster()
#
        #The init will only continue if a transform between parent frame and child frame can be found
        while self.__tfBuffer.can_transform(self.parent_frame, self.child_frame, rospy.Time()) == 0:
            try:
                rospy.loginfo("No transform between "+str(self.parent_frame) +' and ' + str(self.child_frame))
                rospy.sleep(2)
            except: #, tf2_ros.ExtrapolationException  (tf2_ros.LookupException, tf2_ros.ConnectivityException)
                rospy.sleep(2)
                continue
        
        rospy.loginfo("Transform between "+str(self.parent_frame) +' and ' + str(self.child_frame) + 'found.')
    
    def update_mission(self, mission):
        self.current_state = mission.data

    def cv_image_publisher(self, publisher, image, msg_encoding="bgra8"):
        """
        Takes a cv::Mat image object, converts it into a ROS Image message type, and publishes it using the specified publisher.
        """
        msgified_img = self.bridge.cv2_to_imgmsg(image, encoding=msg_encoding)
        publisher.publish(msgified_img)
    
    def publish_waypoint(self, publisher, objectID, waypoint):
        p = ObjectPosition()
        #p.pose.header[]
        p.objectID = objectID
        try:
            p.objectPose.header.seq += 1
        except AttributeError:
            p.objectPose.header.seq = 1
            
        p.header.time = rospy.get_rostime()
        p.header.frame_id = self.parent_frame

        p.objectPose.pose.position.x = waypoint[0]
        p.objectPose.pose.position.y = waypoint[1]
        p.objectPose.pose.position.z = waypoint[2]
        p.objectPose.pose.orientation.x = 0
        p.objectPose.pose.orientation.y = 0
        p.objectPose.pose.orientation.z = 0
        p.objectPose.pose.orientation.w = 1

        p.isDetected = self.isDetected
        p.estimateConverged = self.estimateConverged
        p.estimateFucked = self.estimateFucked

        publisher.publish(p)
        rospy.loginfo("Object published: %s", objectID)

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

    def get_dp_ref(self, path_centroid_camera, z_udfc_odom):
        """
        Takes in the path centroid in homogenious camera coordinates and calculates the X, Y, Z for the dp reference

        Input: path_centroid in homogenious camera coordinates [x_tilde, y_tilde]^T

        Output: path_centroid in odom frame [X, Y, Z]
        """

        # Find out how far above path we are
        z_over_path = self.H_pool_prior - z_udfc_odom - self.h_path_prior
        
        # Map X and Y to world frame
        X = (z_over_path / self.focal_length) * path_centroid_camera[0]
        Y = (z_over_path / self.focal_length) * path_centroid_camera[1]

        # Set dp_ref to be path centroid in x and y and the desired Z to get view of centroid
        dp_ref = [X, Y, self.Z_prior_ref]
        
        return dp_ref

    def find_contour_line(self, contour, img_drawn=None):
        """
        Finds the line in image coordinates, and computes two points that are later to be stored in world.

        Output:         p0 - a point on the line
                        p1 - another point in the line, exactly [vx, vy] along the line from p0, s.t. colin_vec = p_line - p0
        """
        vx, vy, x0, y0 = cv2.fitLine(contour, cv2.DIST_L2, 0, 0.1, 0.1)

        colin_vec = np.ravel(np.array((vx, vy)))
        p0 = np.ravel(np.array((x0, y0)))
        p_line = p0 + colin_vec
        
        if img_drawn is not None:
            p1 = p0 + 1000*colin_vec
            p2 = p0 - 1000*colin_vec
            cv2.line(img_drawn, tuple(p1), tuple(p2), color=(0, 255, 0), thickness=2)

            return p_line, p0, img_drawn
        
        else:
            return p_line, p0

    def batch_estimate_waypoint(self):
        
        line_params = np.average(self.batch_line_params, 0)
        
        p_line      = line_params[:2]
        p0          = line_params[2:]
        
        colin_vec = p_line - p0

        # Get two points on the line
        p1 = p0 + 100*colin_vec
        p2 = p0 - 100*colin_vec
        
        points = [p1, p2]
        # 
        errors = [np.linalg.norm(self.y_min_point - points[i]) for i in range(len(points))]
        next_waypoint = points[np.argmin(errors)]
        
        return np.append(next_waypoint, self.Z_prior_ref)
    
    def path_following_udfc_cb(self, img_msg):

        """
        The first time we classify the path in UDFC makes us move to converge state

        Simplest Way to do this: Find biggest contour and if it is above some threshold for area declare the path there

        """
        self.waypoint_header = img_msg.header

        if self.current_state not in self.possible_states:
            return None

        if np.shape(self.batch_line_params)[0] > 100:
            # In case we have gathered enough information
            self.isEstimated = True
            next_waypoint = self.batch_estimate_waypoint()
            self.publish_waypoint(self.wpPub, "next_task", next_waypoint)
            return None

        udfc_img = self.bridge.imgmsg_to_cv2(img_msg, "passthrough")
        
        # Extracting the contour
        self.path_contour, path_area, img_drawn, hsv_val_img = self.path_calculations(udfc_img)
        self.path_contour = self.path_contour[0][:,0,:]
        
        # Contour detection
        if self.isDetected == False and path_area > self.detection_area_threshold:
            self.isDetected == True
        
        # TODO: test if this gives same result as first finding the centroid in image and them mapping it
        #path_contour_camera = cv2.undistortPoints(path_contour, cameraMatrix=self.camera_matrix, dts=self.udfc_dcs)

        # Get centroid of contour in image coordinates
        M = cv2.moments(self.path_contour)
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        
        self.path_centroid = np.array([cx, cy])
        
        # Undistort centroid point
        path_centroid_cam = cv2.undistort(self.path_centroid, self.K, self.dcs, None, self.K_opt)
        
        # Get drone position in odom and extract the z coordinate for future computations
        tf_lookup_wc = self.__tfBuffer.lookup_transform(self.parent_frame, self.child_frame, rospy.Time(), rospy.Duration(5))
        z_udfc_odom = tf_lookup_wc[0][2]

        dp_ref = self.get_dp_ref(path_centroid_cam, z_udfc_odom)

        # Get the upper contour
        upper_inds = np.where((self.path_contour[:,1] < cy) == True)[0]
        upper_contour_image = self.path_contour[upper_inds]
        
        # Get line, eventually as 2 points in odom and store those
        p_line, p0, img_drawn = self.find_contour_line(upper_contour_image, img_drawn)
        p_line_odom = cv2.undistort(p_line , self.K, self.dcs, None, self.K_opt) + tf_lookup_wc[0]
        p0_odom = cv2.undistort(p0 , self.K, self.dcs, None, self.K_opt) + tf_lookup_wc[0]

        self.batch_line_params = np.vstack((self.batch_line_params, np.reshape(np.append(p_line_odom, p0_odom), (1,4))))

        
        self.cv_image_publisher(self.udfcPub, img_drawn, "bgr8")
        self.publish_waypoint(self.wpPub, "path", dp_ref)


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