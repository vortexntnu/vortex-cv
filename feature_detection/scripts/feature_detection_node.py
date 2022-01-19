#!/usr/bin/env python

import rospy

from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Empty
from cv_msgs.msg import BBox
from darknet_ros_msgs.msg import BoundingBox

from cv_bridge import CvBridge, CvBridgeError
import dynamic_reconfigure.client

import numpy as np
from timeit import default_timer as timer

from feature_detection import FeatureDetection

class FeatureDetectionNode():
    """Handles tasks related to feature detection
    """

    def __init__(self):
        rospy.init_node('feature_detection_node')

        self.zedSub                 = rospy.Subscriber('/zed2/zed_node/rgb/image_rect_color', Image, self.zed_callback)
        self.resetSub               = rospy.Subscriber('/feature_detection/reset', Empty, self.feature_detection_reset_callback)
        # self.fsmStateSub            = rospy.Subscriber('/AUTONOMOUS/FSM_STATE', MISSING_TYPE, self.FSM_cb)

        self.hsvCheckPub            = rospy.Publisher('/feature_detection/hsv_check_image', Image, queue_size= 1)
        self.noiseRmPub             = rospy.Publisher('/feature_detection/noise_removal_image', Image, queue_size= 1)
        self.shapePub               = rospy.Publisher('/feature_detection/shapes_image', Image, queue_size= 1)
        self.linesPub               = rospy.Publisher('/feature_detection/lines_image', Image, queue_size= 1)
        self.BBoxPub                = rospy.Publisher('/feature_detection/bbox_image', Image, queue_size= 1)
        
        self.BBoxPointsPub          = rospy.Publisher('/feature_detection/detection_bbox', BoundingBox, queue_size= 1)
        # self.RectPointsPub          = rospy.Publisher('/feature_detection/object_points', SOMETOPIC, queue_size= 1)

        self.timerPub               = rospy.Publisher('/feature_detection/fps_timer', Float32, queue_size= 1)

        self.bridge = CvBridge()

        self.ref_points_initial_guess = np.array([[449, 341], [845, 496], [690, 331]], dtype=int)

        self.feat_detection = None
        
        # Canny params
        self.canny_threshold1 = 100
        self.canny_threshold2 = 200
        self.canny_aperture = 3

        # HSV params
        self.hsv_hue_min = 179
        self.hsv_hue_max = 0
        self.hsv_sat_min = 255
        self.hsv_sat_max = 0
        self.hsv_val_min = 255
        self.hsv_val_max = 0
        self.hsv_params = [self.hsv_hue_min, self.hsv_hue_max, self.hsv_sat_min, self.hsv_sat_max, self.hsv_val_min, self.hsv_val_max]

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

        self.dynam_client = dynamic_reconfigure.client.Client("feature_detection_cfg", timeout=5.0, config_callback=self.dynam_reconfigure_callback)


    def cv_image_publisher(self, publisher, image, msg_encoding="bgra8"):
        """
        Takes a cv::Mat image object, converts it into a ROS Image message type, and publishes it using the specified publisher.
        """
        msgified_img = self.bridge.cv2_to_imgmsg(image, encoding=msg_encoding)
        publisher.publish(msgified_img)


    def zed_callback(self, img_msg):
        start = timer() # Start function timer.
        #------------------------------------------>
        #------------------------------------------>
        #------------------------------------------>
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, "passthrough")
        except CvBridgeError, e:
            rospy.logerr("CvBridge Error: {0}".format(e))
        self.img_height, self.img_width, self.img_channels = cv_image.shape

        if self.feat_detection == None:
            self.feat_detection = FeatureDetection(self.feat_detection, icp_ref_points=self.ref_points_initial_guess)
        else:
            self.feat_detection.classification(cv_image, "OBJECT FROM FSM", self.hsv_params, self.noise_rm_params)

        self.cv_image_publisher(self.hsvCheckPub, self.feat_detection.hsv_mask_validation_img)
        self.cv_image_publisher(self.noiseRmPub, self.feat_detection.noise_removed_img)
        self.cv_image_publisher(self.shapePub, self.feat_detection.rect_filtering_img)
        self.cv_image_publisher(self.linesPub, self.feat_detection.line_fitting_img)
        self.cv_image_publisher(self.BBoxPub, self.feat_detection.bounding_box_image)

        #------------------------------------------>
        #------------------------------------------>
        #------------------------------------------>
        end = timer() # Stop function timer.
        timediff = (end - start)
        fps = 1 / timediff # Take reciprocal of the timediff to get runs per second. 
        self.timerPub.publish(fps)

    def feature_detection_reset_callback(self, msg):
        self.feat_detection.points_processing_reset()

    def dynam_reconfigure_callback(self, config):
        self.canny_threshold1 = config.canny_threshold1
        self.canny_threshold2 = config.canny_threshold2
        self.canny_aperture = config.canny_aperture_size

        self.hsv_hue_min = config.hsv_hue_min
        self.hsv_hue_max = config.hsv_hue_max
        self.hsv_sat_min = config.hsv_sat_min
        self.hsv_sat_max = config.hsv_sat_max
        self.hsv_val_min = config.hsv_val_min
        self.hsv_val_max = config.hsv_val_max

        self.ksize1 = config.ksize1
        self.ksize2 = config.ksize2
        self.sigma = config.sigma

        self.thresholding_blocksize = config.blocksize
        self.thresholding_C = config.C

        self.erosion_dilation_ksize = config.ed_ksize
        self.erosion_iterations = config.erosion_iterations
        self.dilation_iterations = config.dilation_iterations


if __name__ == '__main__':
    node = FeatureDetectionNode()

    while not rospy.is_shutdown():
        rospy.spin()

