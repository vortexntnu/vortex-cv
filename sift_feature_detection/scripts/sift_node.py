#!/usr/bin/env python

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
        ##################
        ##### Params #####
        ##################

        # Image rate 
        self.get_image_rate = 0.1

        #  grayscale = 0, color = 1
        self.colormode = 1

        # Compare image(s)
        self.image_list = [cv.imread(file, self.colormode) for file in glob.glob("/home/vortex/cv_ws/src/Vortex-CV/sift_feature_detection/data/sift_images/*.png")]

        # Minimum matches needed for drawing bounding box
        self.MIN_MATCH_COUNT = 8

        # Scale the bounding box (Only downscale)
        self.scale = 0.999

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

        ################
        ###CV stuff ####
        ################
        self.bridge = CvBridge() 

        # Initiate SIFT detector (opencv version 3.4.x) 
        self.sift = cv.xfeatures2d.SIFT_create()
        # (opencv version 4.5.1)
        #self.sift = cv.SIFT_create()

        self.kp = []
        self.des = []

        rospy.loginfo("Number of images: %s", len(self.image_list))

        # Gets keypoints and descriptors for every loaded image
        for image in self.image_list:
            k, d = self.sift.detectAndCompute(image,None)
            #rospy.loginfo("print k %s", k)
            self.kp.append(k)
            self.des.append(d)

        ############
        ##Init end##
        ############

    def drawcircle(self, image, center_coordinates=(10, 20), color=(255, 0, 0), radius=5, thickness=2):
        image = cv.circle(image, center_coordinates, radius, color, thickness)
        return image 

    def draw_all_circles(self, dst, radius):

        points = np.reshape(dst, (4,2))
        self.drawcircle(self.cv_image, center_coordinates=(points[0][0], points[0][1]), color=(255, 0, 0),     radius=radius)
        self.drawcircle(self.cv_image, center_coordinates=(points[1][0], points[1][1]), color=(0, 255, 0),     radius=radius)
        self.drawcircle(self.cv_image, center_coordinates=(points[2][0], points[2][1]), color=(0, 0, 255),     radius=radius)
        self.drawcircle(self.cv_image, center_coordinates=(points[3][0], points[3][1]), color=(255, 255, 255), radius=radius)

    def text_on_image(self, img, dst, image_number):
        points = np.reshape(dst, (4,2))
        x, y = int(points[0][0]) ,int(points[0][1] - 15.0)
        h,w = img.shape[0], img.shape[1]
        text = "image" + str(image_number)

        cv.putText(img, text, (x, y), cv.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2, cv.LINE_AA)

    def draw_things(self, img, dst, dst_packed, centeroid, image_number):
        self.draw_all_circles(dst, radius=8)
        self.draw_all_circles(dst_packed, radius=5)
        self.text_on_image(self.cv_image, dst, image_number)
        self.drawcircle(self.cv_image, center_coordinates=(centeroid[0], centeroid[1]))
        self.cv_image = cv.polylines(self.cv_image,[np.int32(dst)],True,255,3, cv.LINE_AA)

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

    def find_centeroid(self, dst):
        '''
        Finds the centeroid for a trapezoid.
        Taking in four corner points and finding the middle points between these.
        The middle points then gets utillized to find the center of the figure using these.

        p1----p14----p4  
        |             |
        |             |
       p12     c     p34
        |             |
        |             |
        p2----p23-----p3

        Args:
            Four cornerpoints in a (4,1,2) array

        Returns:
            The centeroid points as a (1,2) array
        '''

        points = np.reshape(dst, (4,2))
        x_c = np.average(points[:,0])
        y_c = np.average(points[:,1])
        center = np.array([x_c, y_c])

        return center

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

    def publish_centeroid(self, img_numb, centeroid, orientation):
        pub = PoseStamped()
        pub.header.frame_id = "image" + str(img_numb)
        pub.pose.position.x = centeroid[0]
        pub.pose.position.y = centeroid[1]
        pub.pose.position.z = 0
        pub.pose.orientation.x = orientation[0]
        pub.pose.orientation.y = orientation[1]
        pub.pose.orientation.z = orientation[2]
        pub.pose.orientation.w = orientation[3]

        self.detection_centeroid_pub.publish(pub)



    def callback(self, img_in):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(img_in, "passthrough")
            if self.colormode == 0:
                self.cv_image = cv.cvtColor(self.cv_image, cv.COLOR_BGR2GRAY)      
        except CvBridgeError, e:
            rospy.logerr("CvBridge Error: {0}".format(e))

        #rospy.loginfo("image recived")

        kp2, des2 = self.sift.detectAndCompute(self.cv_image,None)
        FLANN_INDEX_KDTREE = 1
        index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
        search_params = dict(checks = 50)
        flann = cv.FlannBasedMatcher(index_params, search_params)
        
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
                M0 = np.zeros((4,4))
                M0[:3, :3] = M[:3, :3]
                M0[3,3] = 1
                orientation = tf.transformations.quaternion_from_matrix(M0)

                # Scales the bounding box
                dst_scaled_cv_packed, dst_scaled = self.scale_bounding_box(dst)
                
                #Finds the centeroid of the bounding box
                centeroid = self.find_centeroid(dst_scaled)

                #self.publish_centeroid(i, centeroid, orientation)
                self.publish_point_array_msg(dst_scaled, "image" + str(i), self.cv_image.shape[1], self.cv_image.shape[0])

                # Only for visual effects (draws bounding box, cornerpoints, etc...)
                self.draw_things(self.cv_image, dst, dst_scaled_cv_packed, centeroid, i)

            else:
                #print( "Not enough matches are found - {}/{}".format(len(good), self.MIN_MATCH_COUNT) )
                matchesMask = None

        if self.colormode == 0:
            pub_img = self.bridge.cv2_to_imgmsg(self.cv_image, encoding="mono8")
        else:
            pub_img = self.bridge.cv2_to_imgmsg(self.cv_image, encoding="bgra8")
        self.detections_pub.publish(pub_img)
        rospy.sleep(self.get_image_rate)
        

if __name__ == '__main__':
    feature = SiftFeature()
    while not rospy.is_shutdown():     
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            pass
    
