#!/usr/bin/env python

# import debugpy
# print("Waiting for VSCode debugger...")
# debugpy.listen(5678)
# debugpy.wait_for_client()

from re import T
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

        self.finished = False

        self.ros_rate = rospy.Rate(3.0)
        # self.zedSub                 = rospy.Subscriber(image_topic, Image, self.camera_callback)

        left_img_topic = "/zed2/zed_node/left_raw/image_raw_color"
        right_img_topic = "/zed2/zed_node/right_raw/image_raw_color"

        self.chessboardSize = (5,7)
        self.criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        self.image_counter = 0

        self.load_config_path = rospy.get_param("/load_config_path")
        self.save_config_path = rospy.get_param("/save_config_path")

        #self.load_config_path = "/home/vortex/vortex_ws/src/Vortex_CV/camera_calibration/scripts/SN38762967FACTORY_REAL_THIS_TIME_FU_BENJAMIN.conf"

        self.objp = np.zeros((self.chessboardSize[0] * self.chessboardSize[1], 3), np.float32)
        self.objp[:,:2] = np.mgrid[0:self.chessboardSize[0],0:self.chessboardSize[1]].T.reshape(-1,2)
        self.objp *= 108 # Chessboard width

        # Arrays to store object points and image points from all the images.
        self.objpoints = [] # 3d point in real world space
        self.imgpointsL = [] # 2d points in image plane.
        self.imgpointsR = [] # 2d points in image plane.

        self.newCameraMatrixL = None
        self.newCameraMatrixR = None
        self.distL = None
        self.distR = None
        self.roi_L = None
        self.roi_R = None

        self.zedLeftSub = rospy.Subscriber(left_img_topic, Image, self.camera_callback_left)
        self.zedRightSub = rospy.Subscriber(right_img_topic, Image, self.camera_callback_right)

        self.calibLeftPub = rospy.Publisher("camera_calibration/rectified/left", Image, queue_size= 10)
        self.calibRightPub = rospy.Publisher("camera_calibration/rectified/right", Image, queue_size= 10)
        
        self.timerPub = rospy.Publisher('/feature_detection/fps_timer', Float32, queue_size= 1)

        self.bridge = CvBridge()

        # First initialization of image shape
        self.first_image_left_msg = rospy.wait_for_message(left_img_topic, Image)
        self.first_image_right_msg = rospy.wait_for_message(right_img_topic, Image)


        try:
            self.cv_image_left = self.bridge.imgmsg_to_cv2(self.first_image_left_msg, "passthrough")
            self.image_shape_left = self.cv_image_left.shape
        except CvBridgeError:
            self.image_shape_left = (720, 1280, 4)


        try:
            self.cv_image_right = self.bridge.imgmsg_to_cv2(self.first_image_right_msg, "passthrough")
            self.image_shape_right = self.cv_image_right.shape
        except CvBridgeError:
            self.image_shape_right = (720, 1280, 4)

        
        # self.dynam_client = dynamic_reconfigure.client.Client("/CVOD/feature_detection_cfg", config_callback=self.dynam_reconfigure_callback)

    

    def spin(self):
        while not rospy.is_shutdown():
            if not self.finished:            
                if self.cv_image_left is not None:
                    try:
                        start = timer() # Start function timer.
                        
                        # self.camera_callback_left(self.first_image_left_msg)
                        # self.camera_callback_right(self.first_image_right_msg)
                        self.calibrate()

                        end = timer() # Stop function timer.
                        timediff = (end - start)
                        fps = 1 / timediff # Take reciprocal of the timediff to get runs per second. 
                        self.timerPub.publish(fps)
                    
                    except Exception:
                        print(traceback.format_exc())
                        print(rospy.get_rostime())

                self.ros_rate.sleep()
            else:
                self.rectify_image()

    def camera_callback_left(self, img_msg):
        
        try:
            self.cv_image_left = self.bridge.imgmsg_to_cv2(img_msg, "passthrough")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))


    def camera_callback_right(self, img_msg):

            try:
                self.cv_image_right = self.bridge.imgmsg_to_cv2(img_msg, "passthrough")
            except CvBridgeError as e:
                rospy.logerr("CvBridge Error: {0}".format(e))

    def calibrate(self):

        imgL = self.cv_image_left
        frameSize = imgL.shape[:2]
        rospy.loginfo("Imagesize: " + str(frameSize))


        imgR = self.cv_image_right
        grayL = cv.cvtColor(imgL, cv.COLOR_BGR2GRAY)
        grayR = cv.cvtColor(imgR, cv.COLOR_BGR2GRAY)

        # Find the chess board corners
        retL, cornersL = cv.findChessboardCorners(grayL, self.chessboardSize, None)
        retR, cornersR = cv.findChessboardCorners(grayR, self.chessboardSize, None)

        if retL and retR == True:

            self.objpoints.append(self.objp)

            cornersL = cv.cornerSubPix(grayL, cornersL, (11,11), (-1,-1), self.criteria)
            self.imgpointsL.append(cornersL)

            cornersR = cv.cornerSubPix(grayR, cornersR, (11,11), (-1,-1), self.criteria)
            self.imgpointsR.append(cornersR)

            self.image_counter += 1
            rospy.loginfo("Counter: " + str(self.image_counter))

        ############## CALIBRATION #######################################################
        if self.image_counter == 50:
            rospy.loginfo("Starting calibrating...")
        #  image_counter = 0

        # Left lens
            retL, cameraMatrixL, distL, rvecsL, tvecsL = cv.calibrateCamera(self.objpoints, self.imgpointsL, frameSize, None, None)
            heightL, widthL, channelsL = imgL.shape
            newCameraMatrixL, roi_L = cv.getOptimalNewCameraMatrix(cameraMatrixL, distL, (widthL, heightL), 1, (widthL, heightL))
            
            self.newCameraMatrixL = newCameraMatrixL
            self.distL = distL
            self.roi_L = roi_L

            #Right lens
            retR, cameraMatrixR, distR, rvecsR, tvecsR = cv.calibrateCamera(self.objpoints, self.imgpointsR, frameSize, None, None)
            heightR, widthR, channelsR = imgR.shape
            newCameraMatrixR, roi_R = cv.getOptimalNewCameraMatrix(cameraMatrixR, distR, (widthR, heightR), 1, (widthR, heightR))
            
            self.newCameraMatrixR = newCameraMatrixR
            self.distR = distR
            self.roi_R = roi_R
            ########## Stereo Vision Calibration #############################################

            flags = 0
            flags |= cv.CALIB_FIX_INTRINSIC

            retStereo, newCameraMatrixL, distL, newCameraMatrixR, distR, rot, trans, essentialMatrix, fundamentalMatrix = cv.stereoCalibrate(self.objpoints, self.imgpointsL, self.imgpointsR, newCameraMatrixL, distL, newCameraMatrixR, distR, grayL.shape[::-1], criteria=self.criteria, flags=flags)
            
            rospy.loginfo("Updating config file...")
            self.update_config(newCameraMatrixL, distL, newCameraMatrixR, distR, str((frameSize[1], frameSize[0])))

        #self.calibPub.publish(left_calib_string)


    def update_config(self, newCameraMatrixL, distL, newCameraMatrixR, distR, resolution):

        #print("Using resolution: ", resolution)
        resolutions = {"(2560,1440)": "LEFT_CAM_2K","(1920,1080)": "[LEFT_CAM_FHD]", "(1280, 720)":"[LEFT_CAM_HD]", "(672, 376)":"[LEFT_CAM_VGA]"}
        
        try:
            # if (resolutions[resolution] == None):
            #     print("No such resolution")
            #     raise Exception("No such resolution")

            rospy.loginfo(resolutions[resolution])
            with open(self.load_config_path, "r") as c:
                config = c.readlines()
        
            j = 0
            for i in range(len(config)):
                if config[i].strip() == resolutions[resolution]:
                    j = i
                    found = True

            resolution_string = resolutions[resolution][10:]

            if found:
                j += 1
                config[j] = "fx={}\n".format(newCameraMatrixL[0][0])
                j += 1
                config[j] = "fy={}\n".format(newCameraMatrixL[1][1])
                j += 1
                config[j] = "cx={}\n".format(newCameraMatrixL[0][2])
                j += 1
                config[j] = "cy={}\n".format(newCameraMatrixL[1][2])
                j += 1
                config[j] = "k1={}\n".format(distL[0][0])
                j += 1
                config[j] = "k2={}\n".format(distL[0][1])
                j += 1
                config[j] = "k3={}\n".format(distL[0][4])
                j += 1
                config[j] = "p1={}\n".format(distL[0][2])
                j += 1
                config[j] = "p2={}\n".format(distL[0][3])
                j += 1
                config[j] = "\n"
                j += 1
                config[j] = "[RIGHT_CAM_{}\n".format(resolution_string)
                j += 1
                config[j] = "fx={}\n".format(newCameraMatrixR[0][0])
                j += 1
                config[j] = "fy={}\n".format(newCameraMatrixR[1][1])
                j += 1
                config[j] = "cx={}\n".format(newCameraMatrixR[0][2])
                j += 1
                config[j] = "cy={}\n".format(newCameraMatrixR[1][2])
                j += 1
                config[j] = "k1={}\n".format(distR[0][0])
                j += 1
                config[j] = "k2={}\n".format(distR[0][1])
                j += 1
                config[j] = "k3={}\n".format(distR[0][4])
                j += 1
                config[j] = "p1={}\n".format(distR[0][2])
                j += 1
                config[j] = "p2={}\n".format(distR[0][3])
                j += 1
                config[j] = "\n"
                #REMOVE THIS
                print("It works")
                

            with open(self.save_config_path, "w") as f:
                for line in config:
                    f.write(line)
                f.close()
            for line in config:
                print(line)

            self.finished = True

            rospy.loginfo("Config file has been saved in: " + self.save_config_path)

        except:
            rospy.logerr("An error occurred")
            rospy.logerr("Valid resolutions are:")
            for key in resolutions.keys():
                rospy.logerr(key)
                


    def rectify_image(self):
        imgL = self.cv_image_left
        imgR = self.cv_image_right

        # Rectify
        rect_image_left = cv.undistort(imgL, self.newCameraMatrixL, self.distL)
        rect_image_right = cv.undistort(imgR, self.newCameraMatrixR, self.distR)

        # Crop
        x, y, w, h = self.roi_L
        rect_image_left = rect_image_left[y:y+h, x:x+w]
        rect_image_right = rect_image_right[y:y+h, x:x+w]

        rect_image_left = self.bridge.cv2_to_imgmsg(rect_image_left)
        rect_image_right = self.bridge.cv2_to_imgmsg(rect_image_right)

        self.calibLeftPub.publish(rect_image_left)
        self.calibRightPub.publish(rect_image_right)





if __name__ == '__main__':
    try:
        feature_detection_node = CalibrationNode()
        # rospy.spin()
        feature_detection_node.spin()

    except rospy.ROSInterruptException:
        pass