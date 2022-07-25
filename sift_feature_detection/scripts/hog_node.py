#!/usr/bin/env python

from click import MultiCommand
import cv2 as cv
from sympy import re
from cv_bridge import CvBridge, CvBridgeError
import glob
import numpy as np

#from skimage.io import imread, imshow
from skimage.transform import resize
from skimage.feature import hog
from skimage import exposure
import matplotlib.pyplot as plt

#ROS imports
import rospy
from rospkg import RosPack
import tf.transformations
from sensor_msgs.msg import Image
from cv_msgs.msg import Point2, PointArray
from geometry_msgs.msg import PoseStamped

# Contains visualization tools
from draw_tools import DrawTools


class HOGDescriptor:
    
    def __init__(self):

        ##################
        ##### Params #####
        ##################

        # Image rate 
        self.get_image_rate = 0

        #  grayscale = 0, color = 1
        self.colormode = 0

        #################
        ###### Node #####
        #################

        #Name of the node
        node_name = "hog"

        # ROS node init
        rospy.init_node(node_name)

        #Subscribe topic
        image_sub = "/zed2/zed_node/rgb/image_rect_color"
        
        rospy.Subscriber(image_sub, Image, self.callback)
        
        # Publisher
        self.detections_pub = rospy.Publisher('/feature_detection/hog', Image, queue_size=1)
        #self.shape_detections_pub = rospy.Publisher('/feature_detection/shape', Image, queue_size=1)


        #self.cornerpoints_pub = rospy.Publisher('/feature_detection/sift_object_points', PointArray, queue_size= 1)
        #self.detection_centeroid_pub = rospy.Publisher('/feature_detection/sift_detection_centeroid', PoseStamped, queue_size=1)

        ################
        ## Hu moments ##
        ################

        #rp = RosPack()
        #path = str(rp.get_path('sift_feature_detection')) + '/data/sift_images/'
        #temp_path = glob.glob(path + "gate_shape.png")


        #gate_shape = cv.imread(temp_path, 0)
        #self.gate_shape = cv.cvtColor(gate_shape, cv.COLOR_BGR2GRAY)     

        ################
        ###CV stuff ####
        ################
        self.drawtools = DrawTools()

        self.bridge = CvBridge() 

        #self.hog = cv.HOGDescriptor()

        rospy.loginfo("legggoooooo")
        
        ############
        ##Init end##
        ############

    def do_hog(self, img):
        _, hog_image = hog(img, orientations=9, pixels_per_cell=(8, 8), 
                    cells_per_block=(2, 2), block_norm = "L2-Hys", visualize=True, multichannel=False)

        hog_image_rescaled = exposure.rescale_intensity(hog_image, in_range=(3, 10))
        rospy.loginfo(np.max(hog_image_rescaled))

        #hog_image_rescaled = hog_image
        #print(np.max(hog_image_rescaled))
        img_ones = self.non_zero_to_one(hog_image_rescaled, threshold=0.5)
        img_ones = np.array(hog_image_rescaled * 255, dtype = np.uint8)

        #ret, hog_image_threshed  = cv.threshold(hog_image_rescaled,0.3,255,cv.THRESH_BINARY)

        return img_ones

    def non_zero_to_one(self, image_array, threshold=0.2):

        arr = np.copy(image_array)
        arr[arr > threshold] = 1
        arr[arr < threshold] = 0

        return arr

    def callback(self, cam_image_ros):                

        try:
            cam_image = self.bridge.imgmsg_to_cv2(cam_image_ros, "passthrough")
            if self.colormode == 0:
                cam_image = cv.cvtColor(cam_image, cv.COLOR_BGR2GRAY)      
        except CvBridgeError, e:
            rospy.logerr("CvBridge Error: {0}".format(e))

        # resized_img = resize(cam_image, (64, 128))

        #resized_img = cam_image

        # hogi = self.hog(resized_img)

        #_, hog_image = hog(resized_img, orientations=9, pixels_per_cell=(8, 8), 
        #            cells_per_block=(2, 2), block_norm = "L2-Hys", visualize=True, multichannel=False)
# 
        # Rescale histogram for better display 
        #hog_image_rescaled = exposure.rescale_intensity(hog_image, in_range=(0, 10)) 

        # im = np.array(hog_image_rescaled * 255, dtype = np.uint8)
        
        im = self.do_hog(cam_image)

        #threshed = cv.adaptiveThreshold(im, 255, cv.ADAPTIVE_THRESH_MEAN_C, cv.THRESH_BINARY, 3, 0)

        cam_image = im
        if self.colormode == 0:
            pub_img = self.bridge.cv2_to_imgmsg(cam_image, encoding="mono8")
        else:
            pub_img = self.bridge.cv2_to_imgmsg(cam_image, encoding="bgra8")
        self.detections_pub.publish(pub_img)

        #rospy.sleep(self.get_image_rate)        

if __name__ == '__main__':
    feature = HOGDescriptor()
    while not rospy.is_shutdown():     
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            pass
    
