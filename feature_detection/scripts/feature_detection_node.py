#!/usr/bin/env python

# import debugpy
# print("Waiting for VSCode debugger...")
# debugpy.listen(5678)
# debugpy.wait_for_client()

import rospy
import rospkg

from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Empty, String
from cv_msgs.msg import Point2, PointArray
from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes

from cv_bridge import CvBridge, CvBridgeError
import dynamic_reconfigure.client

import numpy as np
import cv2
import math
from timeit import default_timer as timer
import traceback

# feature detection library
from feature_detection import FeatureDetection
from read_yaml_config import read_yaml_file

from time import sleep
import copy

from test_gate_detection_cv_original import HoughMajingo

class FeatureDetectionNode():
    """Handles tasks related to feature detection
    """

    def __init__(self, image_topic):
        rospy.init_node('feature_detection_node')
        self.rospack = rospkg.RosPack()
        
        feat_det_pkg_path = self.rospack.get_path('feature_detection')
        self.gate_cfg_path = feat_det_pkg_path + "/object_cfgs/gate_2.yaml"
        self.pole_cfg_path = feat_det_pkg_path + "/object_cfgs/pole_cfg.yaml"

        self.ros_rate = rospy.Rate(60.0)

        self.zedSub                 = rospy.Subscriber(image_topic, Image, self.camera_callback)
        self.resetSub               = rospy.Subscriber('/feature_detection/reset', Empty, self.feature_detection_reset_callback)
        self.fsmStateSub            = rospy.Subscriber('/fsm/state', String, self.fsm_state_cb)

        self.hsvCheckPub            = rospy.Publisher('/feature_detection/hsv_check_image', Image, queue_size= 1)
        self.noiseRmPub             = rospy.Publisher('/feature_detection/noise_removal_image', Image, queue_size= 1)
        self.i2rcpPub               = rospy.Publisher('/feature_detection/i2rcp_image', Image, queue_size= 1)
        self.shapePub               = rospy.Publisher('/feature_detection/shapes_image', Image, queue_size= 1)
        self.linesPub               = rospy.Publisher('/feature_detection/lines_image', Image, queue_size= 1)
        self.pointAreasPub          = rospy.Publisher('/feature_detection/point_areas_image', Image, queue_size= 1)
        self.BBoxPub                = rospy.Publisher('/feature_detection/bbox_image', Image, queue_size= 1)
        
        self.BBoxPointsPub          = rospy.Publisher('/feature_detection/detection_bbox', BoundingBoxes, queue_size= 1)
        self.RectPointsPub          = rospy.Publisher('/feature_detection/object_points', PointArray, queue_size= 1)

        self.timerPub               = rospy.Publisher('/feature_detection/fps_timer', Float32, queue_size= 1)

        self.bridge = CvBridge()

        # ICP initial reference points
        self.ref_points_initial_guess = np.array([[449, 341], [845, 496], [690, 331]], dtype=int)

        # FSM
        self.current_object = "UNKNOWN"
        self.fsm_state = "UNKNOWN"
        self.prev_fsm_state = "UNKNOWN" # Not used rn
        self.states_obj_dict = {"gate_search": 'gate', "pole_search": 'pole'}
        self.states_cfg_dict = {"gate_search": self.gate_cfg_path, "pole_search": self.pole_cfg_path}

        # Canny params
        self.canny_threshold1 = 100
        self.canny_threshold2 = 200
        self.canny_aperture = 3

        # HSV params
        self.hsv_params = [0,
                           179,
                           0,
                           255,
                           0,
                           255]

        # Blur params
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
        
        # First initialization of image shape
        first_image_msg = rospy.wait_for_message(image_topic, Image)
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(first_image_msg, "passthrough")
            self.image_shape = self.cv_image.shape
        except CvBridgeError, e:
            self.image_shape = (720, 1280, 4)

        self.feat_detection = FeatureDetection(self.image_shape, icp_ref_points=self.ref_points_initial_guess)
        
        self.dynam_client = dynamic_reconfigure.client.Client("/CVOD_cfg/feature_detection_cfg", config_callback=self.dynam_reconfigure_callback)

    def cv_image_publisher(self, publisher, image, msg_encoding="bgra8"):
        """
        Takes a cv::Mat image object, converts it into a ROS Image message type, and publishes it using the specified publisher.
        """
        msgified_img = self.bridge.cv2_to_imgmsg(image, encoding=msg_encoding)
        publisher.publish(msgified_img)
    
    def build_bounding_boxes_msg(self, bbox_points, obj_class):
        bbox = BoundingBox()
        bbox.probability = 69.69
        bbox.xmin = bbox_points[0]
        bbox.ymin = bbox_points[1]
        bbox.xmax = bbox_points[2]
        bbox.ymax = bbox_points[3]
        bbox.z = 100000.0
        bbox.id = 0
        bbox.Class = obj_class

        new_bbox_points_msg = BoundingBoxes()
        new_bbox_points_msg.header.stamp = rospy.get_rostime()
        new_bbox_points_msg.header.frame_id = "zed_left_optical_camera_sensor"
        new_bbox_points_msg.bounding_boxes.append(bbox)

        return new_bbox_points_msg
    
    def build_point_array_msg(self, point_array, obj_class, image_width, image_height):
        pt_arr_msg = PointArray()

        for pt_idx in range(len(point_array)):
            pt = point_array[pt_idx]
            pt2_msg = Point2()
            pt2_msg.x = pt[0]
            pt2_msg.y = pt[1]

            pt_arr_msg.point_array.append(pt2_msg)

        pt_arr_msg.header.stamp = rospy.get_rostime()
        pt_arr_msg.header.frame_id = "zed_left_optical_camera_sensor"
        pt_arr_msg.Class = obj_class
        pt_arr_msg.width = image_width
        pt_arr_msg.height = image_height
        
        return pt_arr_msg
    
    def hough_transform_manifold(self):
        cv_img_cp = copy.deepcopy(self.cv_image)
        canny_img = cv2.Canny(cv_img_cp, self.canny_threshold1, self.canny_threshold2, apertureSize=self.canny_aperture)
                    
        lines = cv2.HoughLines(canny_img, 1, np.pi / 180, 150, None, 0, 0)
        cnny_clors = cv2.cvtColor(canny_img, cv2.COLOR_GRAY2BGR)
        
        if lines is not None:
            for i in range(0, len(lines)):
                rho = lines[i][0][0]
                theta = lines[i][0][1]
                a = math.cos(theta)
                b = math.sin(theta)
                x0 = a * rho
                y0 = b * rho
                pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
                pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
                cv2.line(cnny_clors, pt1, pt2, (0,0,255), 3, cv2.LINE_AA)
                cv2.line(cv_img_cp, pt1, pt2, (0,0,255), 3, cv2.LINE_AA)

        return cv_img_cp, cnny_clors, lines

    def spin(self):
        while not rospy.is_shutdown():            
            if self.cv_image is not None:
                try:
                    start = timer() # Start function timer.

                    # cv2_image = cv2.convertScaleAbs(self.cv_image, alpha=self.alpha, beta=self.beta)
                    # lookUpTable = np.empty((1,256), np.uint8)
                    # for i in range(256):
                    #     lookUpTable[0,i] = np.clip(pow(i / 255.0, self.gamma) * 255.0, 0, 255)
                    # res = cv2.LUT(cv2_image, lookUpTable)
                    # self.cv_image_publisher(self.hsvCheckPub, res)
                    t1 = 80 # 100
                    t2 = 150 # 200
                    cv_img_cp = copy.deepcopy(self.cv_image)
                    cropped_image = cv_img_cp[5:720, 0:1280]

                    bb_arr, center, hough_img, edges = HoughMajingo.main(cropped_image, t1, t2)   # self.canny_threshold1, self.canny_threshold2, self.cv_image
                    self.cv_image_publisher(self.linesPub, hough_img) 
                    self.cv_image_publisher(self.i2rcpPub, edges, '8UC1')
                    
                    # if len(bb_arr) != 0:
                    #     rospy.loginfo("Now detecting: bounding box %s", bb_arr)
                    #     rospy.loginfo( center)

                    end = timer() # Stop function timer.
                    timediff = (end - start)
                    fps = 1 / timediff # Take reciprocal of the timediff to get runs per second. 
                    self.timerPub.publish(fps)
                
                except Exception, e:
                    rospy.logwarn(traceback.format_exc())
                    rospy.logwarn(rospy.get_rostime())

            self.ros_rate.sleep()

    def camera_callback(self, img_msg):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(img_msg, "passthrough")
        except CvBridgeError, e:
            rospy.logerr("CvBridge Error: {0}".format(e))
    
    def fsm_state_cb(self, state_msg):
        if state_msg.data in self.states_obj_dict:
            self.fsm_state = state_msg.data
            self.current_object = self.states_obj_dict[self.fsm_state]

            config_path = self.states_cfg_dict[self.fsm_state]
            self.load_obj_config(config_path)

            rospy.loginfo("Now detecting: %s", self.current_object)

            self.feat_detection.points_processing_reset()
    
    def load_obj_config(self, config_path):
        params = read_yaml_file(config_path)

        self.hsv_params[0] = params['hsv_hue_min']
        self.hsv_params[1] = params['hsv_hue_max']
        self.hsv_params[2] = params['hsv_sat_min']
        self.hsv_params[3] = params['hsv_sat_max']
        self.hsv_params[4] = params['hsv_val_min']
        self.hsv_params[5] = params['hsv_val_max']

        self.noise_rm_params = [params["ksize1"],
                                params["ksize2"],
                                params["sigma"],
                                params["blocksize"],
                                params["C"], 
                                params["ed_ksize"], 
                                params["erosion_iterations"], 
                                params["dilation_iterations"]]
    
    def feature_detection_reset_callback(self, msg):
        self.feat_detection.points_processing_reset()

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
        feature_detection_node = FeatureDetectionNode(image_topic='/cv/preprocessing/image_rect_color_filtered')  # /cv/image_preprocessing/CLAHE  /zed2/zed_node/rgb/image_rect_color
        # rospy.spin()
        feature_detection_node.spin()

    except rospy.ROSInterruptException:
        pass