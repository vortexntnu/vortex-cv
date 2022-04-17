#!/usr/bin/env python

# import debugpy
# print("Waiting for VSCode debugger...")
# debugpy.listen(5678)
# debugpy.wait_for_client()

import rospy

from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Empty
from cv_msgs.msg import BBox
from darknet_ros_msgs.msg import BoundingBox

from cv_bridge import CvBridge, CvBridgeError
import dynamic_reconfigure.client

import numpy as np
from timeit import default_timer as timer
import traceback

# feature detection library
from feature_detection import FeatureDetection


class FeatureDetectionNode():
    """Handles tasks related to feature detection
    """

    def __init__(self):
        rospy.init_node('feature_detection_node')

        self.ros_rate = rospy.Rate(30.0)

        self.zedSub                 = rospy.Subscriber('/front/image_view/output', Image, self.camera_callback)
        self.resetSub               = rospy.Subscriber('/feature_detection/reset', Empty, self.feature_detection_reset_callback)
        # self.fsmStateSub            = rospy.Subscriber('/AUTONOMOUS/FSM_STATE', MISSING_TYPE, self.FSM_cb)

        self.hsvCheckPub            = rospy.Publisher('/feature_detection/hsv_check_image', Image, queue_size= 1)
        self.noiseRmPub             = rospy.Publisher('/feature_detection/noise_removal_image', Image, queue_size= 1)
        self.i2rcpPub               = rospy.Publisher('/feature_detection/i2rcp_image', Image, queue_size= 1)
        self.shapePub               = rospy.Publisher('/feature_detection/shapes_image', Image, queue_size= 1)
        self.linesPub               = rospy.Publisher('/feature_detection/lines_image', Image, queue_size= 1)
        self.pointAreasPub          = rospy.Publisher('/feature_detection/point_areas_image', Image, queue_size= 1)
        self.BBoxPub                = rospy.Publisher('/feature_detection/bbox_image', Image, queue_size= 1)
        
        self.BBoxPointsPub          = rospy.Publisher('/feature_detection/detection_bbox', BoundingBox, queue_size= 1)
        # self.RectPointsPub          = rospy.Publisher('/feature_detection/object_points', SOMETOPIC, queue_size= 1)

        self.timerPub               = rospy.Publisher('/feature_detection/fps_timer', Float32, queue_size= 1)

        self.bridge = CvBridge()

        self.ref_points_initial_guess = np.array([[449, 341], [845, 496], [690, 331]], dtype=int)

        # self.image_shape = (720, 1280, 4)
        self.image_shape = (492, 768, 3)
        self.cv_image = None

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
        
        self.feat_detection = FeatureDetection(self.image_shape, icp_ref_points=self.ref_points_initial_guess)
        
        self.dynam_client = dynamic_reconfigure.client.Client("/CVOD_cfg/feature_detection_cfg", config_callback=self.dynam_reconfigure_callback)

    def cv_image_publisher(self, publisher, image, msg_encoding="bgra8"):
        """
        Takes a cv::Mat image object, converts it into a ROS Image message type, and publishes it using the specified publisher.
        """
        msgified_img = self.bridge.cv2_to_imgmsg(image, encoding=msg_encoding)
        publisher.publish(msgified_img)
    
    def spin(self):
        while not rospy.is_shutdown():            
            if self.cv_image is not None:
                try:
                    start = timer() # Start function timer.
                    hsv_img, hsv_mask, hsv_mask_validation_img = self.feat_detection.hsv_processor(self.cv_image, *self.hsv_params)
                    self.cv_image_publisher(self.hsvCheckPub, hsv_mask_validation_img, msg_encoding="bgr8")

                    nr_img = self.feat_detection.noise_removal_processor(hsv_mask, *self.noise_rm_params)
                    self.cv_image_publisher(self.noiseRmPub, nr_img, msg_encoding="mono8")

                    # bbox_points, bbox_area, points_in_rects, detection = self.feat_detection.classification(self.cv_image, "SOMETHING", self.hsv_params, self.noise_rm_params)

                    # self.cv_image_publisher(self.hsvCheckPub, self.feat_detection.hsv_validation_img, msg_encoding="bgr8")
                    # self.cv_image_publisher(self.noiseRmPub, self.feat_detection.nr_img, msg_encoding="mono8")
                    # self.cv_image_publisher(self.i2rcpPub, self.feat_detection.i2rcp_image_blank, msg_encoding="bgr8")
                    # self.cv_image_publisher(self.shapePub, self.feat_detection.rect_flt_img, msg_encoding="bgr8")
                    # self.cv_image_publisher(self.linesPub, self.feat_detection.line_fitting_img, msg_encoding="bgr8")
                    # self.cv_image_publisher(self.BBoxPub, self.feat_detection.bbox_img, msg_encoding="bgr8")
                    # self.cv_image_publisher(self.pointAreasPub, self.feat_detection.pointed_rects_img, msg_encoding="bgr8")

                    end = timer() # Stop function timer.
                    timediff = (end - start)
                    fps = 1 / timediff # Take reciprocal of the timediff to get runs per second. 
                    self.timerPub.publish(fps)
                
                except Exception, e:
                    print(traceback.format_exc())
                    print(rospy.get_rostime())

            self.ros_rate.sleep()

    def camera_callback(self, img_msg):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(img_msg, "passthrough")
        except CvBridgeError, e:
            rospy.logerr("CvBridge Error: {0}".format(e))

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
        feature_detection_node = FeatureDetectionNode()
        # rospy.spin()
        feature_detection_node.spin()

    except rospy.ROSInterruptException:
        pass