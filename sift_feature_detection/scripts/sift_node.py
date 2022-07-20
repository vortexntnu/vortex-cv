#!/usr/bin/env python

from operator import imod
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
import glob
import numpy as np


#ROS imports
import rospy
import tf.transformations
from sensor_msgs.msg import Image
from cv_msgs.msg import Point2, PointArray
from geometry_msgs.msg import PoseStamped

# Contains visualization tools
from draw_tools import DrawTools


class SiftFeature:
    '''
    This node:

    * Reads images from a folder, generates kp (keypoints) and des (descriptors) and stores them for the session

    * Compares stored kp and des with the ones from the camera image.

    * When matches are within the threshold the homography and perspective trasform (opencv functions) gets applied

    * The corner points from the read images gets found in the cam image, aswell as the centeroid


    * Bounding box and found points gets drawn unless commented out

    '''
    
    def __init__(self):

        #################
        ###### Node #####
        #################

        #Name of the node
        node_name = "sift_feature"

        # ROS node init
        rospy.init_node(node_name)

        #Subscribe topic
        image_sub = "/zed2/zed_node/rgb/image_rect_color"
        
        rospy.Subscriber(image_sub, Image, self.callback)
        
        # Publisher
        self.detections_pub = rospy.Publisher('/feature_detection/sift_bbox_image', Image, queue_size=1)

        self.cornerpoints_pub = rospy.Publisher('/feature_detection/sift_object_points', PointArray, queue_size= 1)
        self.detection_centeroid_pub = rospy.Publisher('/feature_detection/sift_detection_centeroid', PoseStamped, queue_size=1)

        ##################
        ##### Params #####
        ##################

        # Image rate 
        self.get_image_rate = 0.1

        #  grayscale = 0, color = 1
        self.colormode = 1

        # Image path
        path = "/home/vortex/cv_ws/src/Vortex-CV/sift_feature_detection/data/sift_images/*.png"

        # Compare image(s)
        self.image_list = [cv.imread(file, self.colormode) for file in glob.glob(path)]
        rospy.loginfo("Number of images: %s", len(self.image_list))
        if len(self.image_list) == 0:
            rospy.logwarn("\n ######################################## \n ### No images found! Check your path!### \n ########################################")

        # Minimum matches needed for drawing bounding box
        self.MIN_MATCH_COUNT = 8

        # Scale the bounding box (Only downscale)
        self.scale = 0.999

        ################
        ###CV stuff ####
        ################
        self.drawtools = DrawTools()

        self.bridge = CvBridge() 

        # Initiate SIFT detector (opencv version 3.4.x) 
        #self.sift = cv.xfeatures2d.SIFT_create()
        # (opencv version 4.5.1)
        self.sift = cv.SIFT_create()

        self.kp = []
        self.des = []

        

        # Gets keypoints and descriptors for every loaded image
        for image in self.image_list:
            k, d = self.sift.detectAndCompute(image,None)
            #rospy.loginfo("print k %s", k)
            self.kp.append(k)
            self.des.append(d)

        ############
        ##Init end##
        ############

    def scale_bounding_box(self, dst):
        '''
        This function downscales a square to a smaller square (no upscale).
        Use small scaling like 0.999 if your bounding box gets crazy when
        choosing a small number.

            p1-----------p4
            | \        /  |
            |   p1s  p4s  |
            |             |
            |   p2s  p3s  |
            |  /        \ |
            p2------------p3

        Returns:
            dst_scaled as a (4,1,2) array and a (4,2) array
        '''
        sf = self.scale - 1
        sf1 = 1 + np.abs(sf)
        sf2 = 1 - np.abs(sf)

        points = np.reshape(dst, (4,2))

        p1 = points[0]
        p2 = points[1]
        p3 = points[2]
        p4 = points[3]
        
        p1 = p1*sf1
        p2[0], p2[1] = p2[0]*sf1, p2[1]*sf2
        p3 = p3*sf2
        p4[0], p4[1] = p4[0]*sf2, p4[1]*sf1 

        dst_scaled = np.array([p1, p2, p3, p4])

        return np.reshape(dst_scaled,(4,1,2)), dst_scaled

    def publish_point_array_msg(self, point_array, obj_class, image_width, image_height):
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

        #rospy.loginfo("Point array published")
        
        self.cornerpoints_pub.publish(pt_arr_msg)

    def compare_matches(self, cam_image, flann, kp2, des2):

        for i in range(len(self.image_list)):
            matches = flann.knnMatch(self.des[i],des2,k=2)

            # store all the good matches as per Lowe's ratio test.
            good = []
            for m,n in matches:
                if m.distance < 0.7*n.distance:
                    good.append(m)

            if len(good)>self.MIN_MATCH_COUNT:
                src_pts = np.float32([ self.kp[i][m.queryIdx].pt for m in good ]).reshape(-1,1,2)
                dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)
                M, mask = cv.findHomography(src_pts, dst_pts, cv.RANSAC,5.0)
                matchesMask = mask.ravel().tolist()

                if len(self.image_list[i].shape) == 2:
                    h,w = self.image_list[i].shape
                else:
                    h, w, _ = self.image_list[i].shape

                pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
                dst = cv.perspectiveTransform(pts,M)

                # Gets orientation of the object
                #M0 = np.zeros((4,4))
                #M0[:3, :3] = M[:3, :3]
                #M0[3,3] = 1
                #orientation = tf.transformations.quaternion_from_matrix(M0)

                # Scales the bounding box
                dst_scaled_cv_packed, dst_scaled = self.scale_bounding_box(dst)
                
                #self.publish_centeroid(i, centeroid, orientation)
                self.publish_point_array_msg(dst_scaled, "image" + str(i), cam_image.shape[1], cam_image.shape[0])

                # Only for visual effects (draws bounding box, cornerpoints, etc...)
                cam_image = self.drawtools.draw_all(cam_image, dst, dst_scaled_cv_packed, i, centeroid=True)

            else:
                #print( "Not enough matches are found - {}/{}".format(len(good), self.MIN_MATCH_COUNT) )
                matchesMask = None

        return cam_image

    def callback(self, cam_image_ros):
        try:
            cam_image = self.bridge.imgmsg_to_cv2(cam_image_ros, "passthrough")
            if self.colormode == 0:
                cam_image = cv.cvtColor(cam_image, cv.COLOR_BGR2GRAY)      
        except CvBridgeError, e:
            rospy.logerr("CvBridge Error: {0}".format(e))

        kp2, des2 = self.sift.detectAndCompute(cam_image, None)
        FLANN_INDEX_KDTREE = 1
        index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
        search_params = dict(checks = 50)
        flann = cv.FlannBasedMatcher(index_params, search_params)
        
        cam_image = self.compare_matches(cam_image, flann, kp2, des2)

        if self.colormode == 0:
            pub_img = self.bridge.cv2_to_imgmsg(cam_image, encoding="mono8")
        else:
            pub_img = self.bridge.cv2_to_imgmsg(cam_image, encoding="bgra8")
        self.detections_pub.publish(pub_img)

        rospy.sleep(self.get_image_rate)
        

if __name__ == '__main__':
    feature = SiftFeature()
    while not rospy.is_shutdown():     
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            pass
    
