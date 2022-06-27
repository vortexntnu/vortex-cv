#!/usr/bin/env python

# import debugpy
# print("Waiting for VSCode debugger...")
# debugpy.listen(5678)
# debugpy.wait_for_client()

import rospy
import rospkg

from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Empty, String, Bool
from cv_msgs.msg import Point2, PointArray
from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes

from cv_bridge import CvBridge, CvBridgeError
import dynamic_reconfigure.client

import numpy as np
from timeit import default_timer as timer
import traceback

# feature detection library
from feature_detection import FeatureDetection
from read_yaml_config import read_yaml_file

from time import sleep

class FeatureDetectionNode():
    """Handles tasks related to feature detection
    """

    def __init__(self, image_topic):
        rospy.init_node('feature_detection_node')
        self.rospack = rospkg.RosPack()
        
        feat_det_pkg_path = self.rospack.get_path('feature_detection')
        self.gate_cfg_path = feat_det_pkg_path + "/object_cfgs/GATE_W_RGB_FILTER.yaml"
        self.pole_cfg_path = feat_det_pkg_path + "/object_cfgs/pole_cfg.yaml"

        self.ros_rate = rospy.Rate(60.0)

        self.zedSub                 = rospy.Subscriber(image_topic, Image, self.camera_callback)
        self.resetSub               = rospy.Subscriber('/feature_detection/reset', Empty, self.feature_detection_reset_callback)
        self.fsmStateSub            = rospy.Subscriber('/fsm/state', String, self.fsm_state_cb)
        self.rcfaSub                = rospy.Subscriber('/cv/preprocessing/rcfa_detection', Bool, self.rcfa_bool_cb)
        self.ocfaSub                = rospy.Subscriber('/cv/preprocessing/ocfa_detection', Bool, self.ocfa_bool_cb)


        self.hsvCheckPub            = rospy.Publisher('/feature_detection/color_filter_check_image', Image, queue_size= 1)
        self.hsvMaskPub             = rospy.Publisher('/feature_detection/color_filter_mask_image', Image, queue_size= 1)
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

        self.rcfa_det = False
        self.ocfa_det = False

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

        # BGR params
        self.bgr_params = [0,
                           255,
                           0,
                           255,
                           0,
                           255]

        # HSV params
        self.hsv_params = [0,
                           179,
                           0,
                           255,
                           1,
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

    def spin(self):
        while not rospy.is_shutdown():
            if self.cv_image is not None:
                try:
                    start = timer() # Start function timer. (0, 113, 0, 97, 19, 120)

                    try:
                        bgr_img, bgr_mask, bgr_mask_validation_img = self.feat_detection.bgr_filter(self.cv_image, *self.bgr_params)
                        _, hsv_mask, hsv_mask_validation_img = self.feat_detection.hsv_processor(bgr_mask_validation_img, *self.hsv_params)
                        nr_img = self.feat_detection.noise_removal_processor(hsv_mask, *self.noise_rm_params)
                    except Exception, e:
                        rospy.logwarn(traceback.format_exc())
                        rospy.logwarn(rospy.get_rostime())

                    try:
                        using_cnts = self.feat_detection.contour_processing(nr_img, 300, return_image=False)
                        shape_img, _, fitted_boxes, centroid_arr = self.feat_detection.shape_fitting(using_cnts, 5, return_image=True, image=bgr_img)
                    except Exception, e:
                        rospy.logwarn(traceback.format_exc())
                        rospy.logwarn(rospy.get_rostime())

                    try:
                        i2rcp_img, i2rcp_image_blank, i2rcp_points = self.feat_detection.i2rcp(centroid_arr, return_image=True, image=bgr_img)
                    except Exception, e:
                        rospy.logwarn(traceback.format_exc())
                        rospy.logwarn(rospy.get_rostime())

                    try:
                        pointed_rects_img, rect_flt_img, _, relevant_rects, points_in_rects = self.feat_detection.rect_filtering(i2rcp_points, fitted_boxes, return_rectangles_separately=False, return_image=True, image=bgr_img)
                        self.rect_flt_img = rect_flt_img
                        pt_arr_msg = self.build_point_array_msg(points_in_rects, self.current_object, self.image_shape[0], self.image_shape[1])
                    except Exception, e:
                        rospy.logwarn(traceback.format_exc())
                        rospy.logwarn(rospy.get_rostime())

                    self.cv_image_publisher(self.hsvCheckPub, hsv_mask_validation_img, "bgr8")
                    self.cv_image_publisher(self.hsvMaskPub, hsv_mask, "mono8")
                    self.cv_image_publisher(self.noiseRmPub, nr_img, msg_encoding="mono8")
                    self.cv_image_publisher(self.i2rcpPub, i2rcp_img, "bgr8")
                    self.cv_image_publisher(self.shapePub, rect_flt_img, "bgr8")
                    self.cv_image_publisher(self.pointAreasPub, pointed_rects_img, "bgr8")

                    self.RectPointsPub.publish(pt_arr_msg)


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
    
    def rcfa_bool_cb(self, msg):
        self.rcfa_det = msg.data

    def ocfa_bool_cb(self, msg):
        self.ocfa_det = msg.data

        
    def load_obj_config(self, config_path):
        params = read_yaml_file(config_path)

        self.bgr_params[0] = params['b_min']
        self.bgr_params[1] = params['b_max']
        self.bgr_params[2] = params['g_min']
        self.bgr_params[3] = params['g_max']
        self.bgr_params[4] = params['r_min']
        self.bgr_params[5] = params['r_max']

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

        self.bgr_params[0] = config.b_min
        self.bgr_params[1] = config.b_max
        self.bgr_params[2] = config.g_min
        self.bgr_params[3] = config.g_max
        self.bgr_params[4] = config.r_min
        self.bgr_params[5] = config.r_max

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
        feature_detection_node = FeatureDetectionNode(image_topic='/cv/image_preprocessing/CLAHE/zed2')
        # rospy.spin()
        feature_detection_node.spin()

    except rospy.ROSInterruptException:
        pass