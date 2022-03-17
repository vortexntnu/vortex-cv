#! /usr/bin/env python

# import debugpy
# print("Waiting for VSCode debugger...")
# debugpy.listen(5678)
# debugpy.wait_for_client()

import rospy

from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError
import dynamic_reconfigure.client

from ImagePreprocessing import ImagePreprocessing
from read_yaml_config import read_yaml_file

import numpy as np
import cv2 as cv

class ImagePreprocessingNode():
    """Performs image preprocessing (duh...)
    """

    def __init__(self, image_topic):
        rospy.init_node('image_preprocessing_node')

        self.zedSub                 = rospy.Subscriber(image_topic, Image, self.image_callback, queue_size= 1)

        self.CLAHEPub               = rospy.Publisher('/cv/image_preprocessing/CLAHE', Image, queue_size= 1)
        self.single_CLAHEPub        = rospy.Publisher('/cv/image_preprocessing/CLAHE_single', Image, queue_size= 1)
        self.GWPub                  = rospy.Publisher('/cv/image_preprocessing/GW', Image, queue_size= 1)
        
        self.bridge = CvBridge()
        self.image_preprocessing = ImagePreprocessing(2, 8)
        
        # First initialization of image shape
        first_image_msg = rospy.wait_for_message(image_topic, Image)

        self.cv_image = self.bridge.imgmsg_to_cv2(first_image_msg, "passthrough")
        self.image_shape = self.cv_image.shape

        self.dynam_client = dynamic_reconfigure.client.Client("/CVOD_cfg/img_preprocessing_cfg", config_callback=self.dynam_reconfigure_callback)

    def cv_image_publisher(self, publisher, image, msg_encoding="bgra8"):
        """
        Takes a cv::Mat image object, converts it into a ROS Image message type, and publishes it using the specified publisher.
        """
        msgified_img = self.bridge.cv2_to_imgmsg(image, encoding=msg_encoding)
        publisher.publish(msgified_img)

    def image_callback(self, img_msg):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(img_msg, "passthrough")
        except CvBridgeError, e:
            rospy.logerr("CvBridge Error: {0}".format(e))

        clahe_img = self.image_preprocessing.CLAHE(self.cv_image)
        clahe_img = clahe_img.round().astype(np.uint8)
        self.cv_image_publisher(self.CLAHEPub, clahe_img, "bgra8")

        _, _, R, _ = cv.split(self.cv_image)
        clahe_r_channel = self.image_preprocessing.CLAHE(R)
        self.cv_image_publisher(self.single_CLAHEPub, clahe_r_channel, "mono8")

        gw_img = self.image_preprocessing.gray_world(cv.cvtColor(self.cv_image, cv.COLOR_BGRA2BGR))
        self.cv_image_publisher(self.GWPub, gw_img, "bgr8")
    
    def load_obj_config(self, config_path):
        params = read_yaml_file(config_path)

        # self.noise_rm_params = [params["ksize1"],
        #                         params["ksize2"],
        #                         params["sigma"],
        #                         params["blocksize"],
        #                         params["C"], 
        #                         params["ed_ksize"], 
        #                         params["erosion_iterations"], 
        #                         params["dilation_iterations"]]

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
        image_preprocessing_node = ImagePreprocessingNode(image_topic='/zed2i/zed_node/rgb/image_rect_color')
        # rospy.spin()
        image_preprocessing_node.spin()

    except rospy.ROSInterruptException:
        pass