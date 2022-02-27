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

        self.chessboardSize = (5,7)
        self.criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        self.image_counter = 0

        #Change this in yaml?
        self.config_path = "/home/vortex/cv_ws/src/Vortex_CV/camera_calibration/scripts/SN38762967FACTORY_REAL_THIS_TIME_FU_BENJAMIN.conf"

        self.objp = np.zeros((self.chessboardSize[0] * self.chessboardSize[1], 3), np.float32)
        self.objp[:,:2] = np.mgrid[0:self.chessboardSize[0],0:self.chessboardSize[1]].T.reshape(-1,2)
        self.objp *= 108 # Chessboard width

        # Arrays to store object points and image points from all the images.
        self.objpoints = [] # 3d point in real world space
        self.imgpointsL = [] # 2d points in image plane.
        self.imgpointsR = [] # 2d points in image plane.

        self.zedLeftSub = rospy.Subscriber(left_img_topic, Image, self.camera_callback_left)
        self.zedRightSub = rospy.Subscriber(right_img_topic, Image, self.camera_callback_right)

        self.calibLeftPub = rospy.Publisher("camera_calibration/left", String, queue_size= 10)
        self.calibRightPub = rospy.Publisher("camera_calibration/right", String, queue_size= 10)
        
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
                    

                    self.camera_callback_left()
                    self.camera_callback_right()
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


        imgL = self.cv_image_left
        frameSize = imgL.shape[:2]

        imgR = self.cv_image_right
        grayL = cv.cvtColor(imgL, cv.COLOR_BGR2GRAY)
        grayR = cv.cvtColor(imgR, cv.COLOR_BGR2GRAY)

        # Find the chess board corners
        retL, cornersL = cv.findChessboardCorners(grayL, self.chessboardSize, None)
        retR, cornersR = cv.findChessboardCorners(grayR, self.chessboardSize, None)

        if retL and retR == True:
            print(imgLeft)

            self.objpoints.append(self.objp)

            cornersL = cv.cornerSubPix(grayL, cornersL, (11,11), (-1,-1), self.criteria)
            self.imgpointsL.append(cornersL)

            cornersR = cv.cornerSubPix(grayR, cornersR, (11,11), (-1,-1), self.criteria)
            self.imgpointsR.append(cornersR)

            self.image_counter += 1

        ############## CALIBRATION #######################################################
        if image_counter == 50:
        #  image_counter = 0

        # Left lens
            retL, cameraMatrixL, distL, rvecsL, tvecsL = cv.calibrateCamera(self.objpoints, self.imgpointsL, frameSize, None, None)
            heightL, widthL, channelsL = imgL.shape
            newCameraMatrixL, roi_L = cv.getOptimalNewCameraMatrix(cameraMatrixL, distL, (widthL, heightL), 1, (widthL, heightL))


            #Right lens
            retR, cameraMatrixR, distR, rvecsR, tvecsR = cv.calibrateCamera(self.objpoints, self.imgpointsR, frameSize, None, None)
            heightR, widthR, channelsR = imgR.shape
            newCameraMatrixR, roi_R = cv.getOptimalNewCameraMatrix(cameraMatrixR, distR, (widthR, heightR), 1, (widthR, heightR))


            ########## Stereo Vision Calibration #############################################

            flags = 0
            flags |= cv.CALIB_FIX_INTRINSIC

            retStereo, newCameraMatrixL, distL, newCameraMatrixR, distR, rot, trans, essentialMatrix, \
            fundamentalMatrix = cv.stereoCalibrate(self.objpoints, self.imgpointsL, self.imgpointsR, newCameraMatrixL, \
            distL, newCameraMatrixR, distR, grayL.shape[::-1], self.criteria, flags)
            

            self.update_config(newCameraMatrixL, distL, newCameraMatrixR, distR, (widthR, heightR)):

        self.calibPub.publish(left_calib_string)


    def update_config(self, newCameraMatrixL, distL, newCameraMatrixR, distR, resolution):

    #print("Using resolution: ", resolution)
    resolutions = {"(2560,1440)": "LEFT_CAM_2K","(1920,1080)": "[LEFT_CAM_FHD]", "(1280, 720)":"[LEFT_CAM_HD]", "(672, 376)":"[LEFT_CAM_VGA]"}
    
    try:
        if (resolutions[resolution] == None):
            print("No such resolution")
            raise Exception("No such resolution")

        print(resolutions[resolution])
        with open(self.config_path, "r") as c:
            config = c.readlines()
    
        j = 0
        for i in range(len(config)):
            if config[i].strip() == resolutions[resolution]:
                j = i
                found = True

        resolution_string = resolutions[resolution][10:]

        if found:
            j += 1
            config[j] = f"fx={newCameraMatrixL[0][0]}\n"
            j += 1
            config[j] = f"fy={newCameraMatrixL[1][1]}\n"
            j += 1
            config[j] = f"cx={newCameraMatrixL[0][2]}\n"
            j += 1
            config[j] = f"cy={newCameraMatrixL[1][2]}\n"
            j += 1
            config[j] = f"k1={distL[0][0]}\n"
            j += 1
            config[j] = f"k2={distL[0][1]}\n"
            j += 1
            config[j] = f"k3={distL[0][4]}\n"
            j += 1
            config[j] = f"p1={distL[0][2]}\n"
            j += 1
            config[j] = f"p2={distL[0][3]}\n"
            j += 1
            config[j] = "\n"
            j += 1
            config[j] = f"[RIGHT_CAM_{resolution_string}\n"
            j += 1
            config[j] = f"fx={newCameraMatrixR[0][0]}\n"
            j += 1
            config[j] = f"fy={newCameraMatrixR[1][1]}\n"
            j += 1
            config[j] = f"cx={newCameraMatrixR[0][2]}\n"
            j += 1
            config[j] = f"cy={newCameraMatrixR[1][2]}\n"
            j += 1
            config[j] = f"k1={distR[0][0]}\n"
            j += 1
            config[j] = f"k2={distR[0][1]}\n"
            j += 1
            config[j] = f"k3={distR[0][4]}\n"
            j += 1
            config[j] = f"p1={distR[0][2]}\n"
            j += 1
            config[j] = f"p2={distR[0][3]}\n"
            j += 1

            config[j] = "\n"

        with open(self.config_path, "w") as f:
            for line in config:
                f.write(line)
            f.close()
        for line in config:
            print(line)

    except:
        print("An error occurred. The function was given an invalid resolution")
        print("Valid resolutions are: \n")
        for key in resolutions.keys():
            print(key)
            
if __name__ == '__main__':
    try:
        feature_detection_node = CalibrationNode()
        # rospy.spin()
        feature_detection_node.spin()

    except rospy.ROSInterruptException:
        pass