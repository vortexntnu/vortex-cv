#!/usr/bin/env python

# import debugpy
# print("Waiting for VSCode debugger...")
# debugpy.listen(5678)
# debugpy.wait_for_client()

import rospy

from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Empty, String
from cv_msgs.msg import BBox
from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes

from cv_bridge import CvBridge, CvBridgeError
import dynamic_reconfigure.client

import numpy as np
from timeit import default_timer as timer
import traceback

import cv2 as cv

class CalibrationNode():
    """Handles tasks related to feature detection
    """

    def __init__(self):
        rospy.init_node('camera_calibration_node')

        self.ros_rate = rospy.Rate(1.0)

        # self.zedSub                 = rospy.Subscriber(image_topic, Image, self.camera_callback)
        
        left_img_topic = "/zed2/zed_node/left_raw/image_raw_color"
        right_img_topic = "/zed2/zed_node/right_raw/image_raw_color"

        self.zedLeftSub = rospy.Subscriber(left_img_topic, Image, self.camera_callback_left)
        self.zedRightSub = rospy.Subscriber(right_img_topic, Image, self.camera_callback_right)

        self.calibPub = rospy.Publisher("camera_calibration/left", String, queue_size= 10)
        
        self.timerPub               = rospy.Publisher('/feature_detection/fps_timer', Float32, queue_size= 1)

        self.bridge = CvBridge()

        # First initialization of image shape
        first_image_left_msg = rospy.wait_for_message(left_img_topic, Image)
        first_image_right_msg = rospy.wait_for_message(right_img_topic, Image)


        try:
            self.cv_image_left = self.bridge.imgmsg_to_cv2(first_image_left_msg, "passthrough")
            self.image_shape_left = self.cv_image_left.shape
        except CvBridgeError, e:
            self.image_shape_left = (720, 1280, 4)


        try:
            self.cv_image_right = self.bridge.imgmsg_to_cv2(first_image_right_msg, "passthrough")
            self.image_shape_right = self.cv_image_right.shape
        except CvBridgeError, e:
            self.image_shape_right = (720, 1280, 4)

        
        # self.dynam_client = dynamic_reconfigure.client.Client("/CVOD/feature_detection_cfg", config_callback=self.dynam_reconfigure_callback)

    

    def spin(self):
        while not rospy.is_shutdown():            
            if self.cv_image is not None:
                try:
                    start = timer() # Start function timer.
                    
                    self.calibrate()

                    end = timer() # Stop function timer.
                    timediff = (end - start)
                    fps = 1 / timediff # Take reciprocal of the timediff to get runs per second. 
                    self.timerPub.publish(fps)
                
                except Exception, e:
                    print(traceback.format_exc())
                    print(rospy.get_rostime())

            self.ros_rate.sleep()

    def camera_callback_left(self, img_msg):
        
        try:
            self.cv_image_left = self.bridge.imgmsg_to_cv2(img_msg, "passthrough")
        except CvBridgeError, e:
            rospy.logerr("CvBridge Error: {0}".format(e))


    def camera_callback_right(self, img_msg):

            try:
                self.cv_image_right = self.bridge.imgmsg_to_cv2(img_msg, "passthrough")
            except CvBridgeError, e:
                rospy.logerr("CvBridge Error: {0}".format(e))

    def calibrate(self):

        # LEFT SIDE
                
        # termination criteria
        # criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        objp = np.zeros((6*8,3), np.float32)
        objp[:,:2] = np.mgrid[0:8,0:6].T.reshape(-1,2)

        objpoints = []
        imgpoints = []

        img = self.cv_image_left
        #gray = cv.imread(filename)
        # gray = cv.cvtColor(self.cv_image_left, cv.COLOR_BGR2GRAY)
    
        # Find the chess board corners
        ret, corners = cv.findChessboardCorners(img, (8,6), None)
        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)
            imgpoints.append(corners)

            #corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)

            # Draw and display the corners
            cv.drawChessboardCorners(img, (8,6), corners, ret)
            
            # Calibration
            ret_left, mtx_left, dist_left, rvecs_left, tvecs_left = cv.calibrateCamera(objpoints, imgpoints,img.shape[::-1], None, None)

            left_calib_string = "ret_left:" + str(ret_left)

        # RIGHT SIDE
                
        # termination criteria
        # criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        objp = np.zeros((6*8,3), np.float32)
        objp[:,:2] = np.mgrid[0:8,0:6].T.reshape(-1,2)

        objpoints = []
        imgpoints = []

        img = self.cv_image_right
        #gray = cv.imread(filename)
        # gray = cv.cvtColor(self.cv_image_left, cv.COLOR_BGR2GRAY)
    
        # Find the chess board corners
        ret, corners = cv.findChessboardCorners(img, (8,6), None)
        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)
            imgpoints.append(corners)

            #corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)

            # Draw and display the corners
            cv.drawChessboardCorners(img, (8,6), corners, ret)
            
            # Calibration
            ret_right, mtx_right, dist_right, rvecs_right, tvecs_right = cv.calibrateCamera(objpoints, imgpoints,img.shape[::-1], None, None)

            right_calib_string = "ret_right: " + str(ret_right)

        self.calibPub.publish(left_calib_string)



if __name__ == '__main__':
    try:
        feature_detection_node = CalibrationNode()
        # rospy.spin()
        feature_detection_node.spin()

    except rospy.ROSInterruptException:
        pass