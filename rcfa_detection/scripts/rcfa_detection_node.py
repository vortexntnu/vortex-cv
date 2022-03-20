#!/usr/bin/env python

from xmlrpc.client import Boolean
import rospy

# msg types
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Float32MultiArray, Bool
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2 as cv
import matplotlib.pyplot as plt


class PreprocessingNode():
    """
    Class to handle operations related to the preprocessing node. \n
    This includes:
        - Confidence map --> masked confidence map
        - and so on !!!!!!!!!!!!!! So on is kind of unspecific
    """
    def __init__(self):
        rospy.init_node('rcfa_detection')

        self.bridge = CvBridge()


        # RCFA_detection
        rospy.Subscriber('/cv/image_preprocessing/GW', Image, self.image_rect_color_cb, queue_size=1)
        self.rcfa_pub = rospy.Publisher('/cv/preprocessing/rcfa_detection',Bool , queue_size= 1)###In order to publish bool value

        # self.x_plot = np.linspace(0, 255., num=256)
        # self.fig_plot = plt.figure()
        # self.ax_plot = self.fig_plot.add_subplot(1, 1, 1)

        # self.line_plot, = self.ax_plot.plot([], lw=3)

        # self.ax_plot.set_xlim(self.x_plot.min(), self.x_plot.max())
        # self.ax_plot.set_ylim([0, 100000])

        # self.fig_plot.canvas.draw()   # note that the first draw comes before setting data 

        # self.ax2background = self.fig_plot.canvas.copy_from_bbox(self.ax_plot.bbox)

        # plt.show(block=True)
    
    
    def orange_boi(self, msg):
        """
        ***Callback***\n
        Publishes a confident representation of the message in the topic using a confidence map.
        Args:
            msg: the message in the topic callback
        """
        cv_image = self.bridge_to_cv(msg)
        
        new_cfd = cv_image.copy()

        # new_cfd[new_cfd < 0.3] = 0
        # new_cfd[new_cfd > 10] = 0

        hist_red = cv.calcHist([new_cfd],[2],None,[256],[0,255]) #One dimensional array. Index of array is the color intensity. The array values are pixels.
        hist_green = cv.calcHist([new_cfd],[1],None,[256],[0,255]) #One dimensional array. Index of array is the color intensity. The array values are pixels.

        hist_full = (hist_red + hist_green*0.6)/1.6

                                                                    # Intensity = 0 is the highest color intensity.
        index = np.argmax(hist_full) #Finds the index of the max element in array.
        max = hist_full[index]  #Finds the maximum value of thr array .

        mean = np.sum(hist_full)/len(hist_full) #Calculates the avarage value of pixels in the array.

        


        difference  = max-mean
        rospy.loginfo(difference)
        #if (difference>20000):
        #    rospy.loginfo("GATE DETECTED")
        #    #rospy.loginfo(difference)
        #    gateDetection = True
        #    self.rcfa_pub.publish(gateDetection) #Publishes gateDetection 


        #else:
        #    rospy.loginfo("NO GATE")
        #    gateDetection = False
        #    self.rcfa_pub.publish(gateDetection)


        #hist_not_full = hist_full[:100]
        
        #thresh = np.sum(hist_not_full)
        #rospy.loginfo(thresh)
        
        
        
        
        
        self.line_plot.set_data(self.x_plot, hist_full)
        # restore background
        self.fig_plot.canvas.restore_region(self.ax2background)

        # redraw just the points
        self.ax_plot.draw_artist(self.line_plot)

        # fill in the axes rectangle
        self.fig_plot.canvas.blit(self.ax_plot.bbox)

        # in this post http://bastibe.de/2013-05-30-speeding-up-matplotlib.html
        # it is mentionned that blit causes strong memory leakage. 
        # however, I did not observe that.

        self.fig_plot.canvas.flush_events()        

    def image_rect_color_cb(self, msg):
        """
        ***Callback***\n
        Publishes a confident representation of the message in the topic using a confidence map.
        Args:
            msg: the message in the topic callback
        """
        cv_image = self.bridge_to_cv(msg)
        
        new_cfd = cv_image.copy()

        # new_cfd[new_cfd < 0.3] = 0
        # new_cfd[new_cfd > 10] = 0

        hist_full = cv.calcHist([new_cfd],[2],None,[256],[0,255]) #One dimensional array. Index of array is the color intensity. The array values are pixels.
                                                                    # Intensity = 0 is the highest color intensity.
        index = np.argmax(hist_full) #Finds the index of the max element in array.
        max = hist_full[index]  #Finds the maximum value of thr array .

        mean = np.sum(hist_full)/len(hist_full) #Calculates the avarage value of pixels in the array.

        


        difference  = max-mean
        rospy.loginfo(difference)
        if (difference>30000):
            rospy.loginfo("GATE DETECTED")
            gateDetection = True
            self.rcfa_pub.publish(gateDetection) #Publishes gateDetection 


        else:
            rospy.loginfo("NO GATE")
            gateDetection = False
            self.rcfa_pub.publish(gateDetection)


        #hist_not_full = hist_full[:100]
        
        #thresh = np.sum(hist_not_full)
        #rospy.loginfo(thresh)
        
        
        
        
        
        # self.line_plot.set_data(self.x_plot, hist_full)
        # # restore background
        # self.fig_plot.canvas.restore_region(self.ax2background)

        # # redraw just the points
        # self.ax_plot.draw_artist(self.line_plot)

        # # fill in the axes rectangle
        # self.fig_plot.canvas.blit(self.ax_plot.bbox)

        # # in this post http://bastibe.de/2013-05-30-speeding-up-matplotlib.html
        # # it is mentionned that blit causes strong memory leakage. 
        # # however, I did not observe that.

        # self.fig_plot.canvas.flush_events()

    def bridge_to_cv(self, image_msg, encoding = "passthrough"):
        """This function returns a cv image from a ros image"""
        # !!!!!!!!!!!!!! comments on arguments are missing
        # Bridge image data from Image to cv_image data
        image_transformed = None
        try:
            image_transformed = self.bridge.imgmsg_to_cv2(image_msg, encoding)
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
        return image_transformed

    def bridge_to_image(self, cv_image_msg, encoding = "passthrough"):
        """This function returns a ros image from a cv image"""
        # comments on arguments are missing
        # Bridge image data from CV image to Image data
        image_transformed = None
        try:
            image_transformed = self.bridge.cv2_to_imgmsg(cv_image_msg, encoding)
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
        return image_transformed
        

if __name__ == '__main__':
    node = PreprocessingNode()

    while not rospy.is_shutdown():
        rospy.spin()
