#!/usr/bin/env python

from xmlrpc.client import Boolean
import rospy

# msg types
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2 as cv
import matplotlib.pyplot as plt


class ncfaDetectionNode():
    """
    Class to detect objects based on color frequency and plot histograms.
    """

    def __init__(self, plot_histogram=False, mode=1):
        '''
        params: 
            plot_histogram (bool): bool value to determine if we will plot a histogarm
            mode (int): Will determine what what channel we will inspect 
                if mode = 1: rcfa activated
                if mode = 2: ocfa activated
        
        '''
        rospy.init_node('rcfa_detection')

        self.bridge = CvBridge()
        self.bool = plot_histogram
        self.mode = mode

        # publisher and subscriber for rcfa
        if (mode == 1):
            rospy.Subscriber('/cv/image_preprocessing/GW/zed2',
                             Image,
                             self.rcfa_cb,
                             queue_size=1)
            self.rcfa_pub = rospy.Publisher('/cv/preprocessing/rcfa_detection',
                                            Bool,
                                            queue_size=1)

        # publisher and subscriber for ocfa
        elif (mode == 2):
            rospy.Subscriber('/cv/image_preprocessing/GW/zed2',
                             Image,
                             self.ocfa_cb,
                             queue_size=1)
            self.ocfa_pub = rospy.Publisher('/cv/preprocessing/ocfa_detection',
                                            Bool,
                                            queue_size=1)

        # Plotting the figure for histogram
        if (self.bool):
            self.x_plot = np.linspace(0, 255., num=256)
            self.fig_plot = plt.figure()
            self.ax_plot = self.fig_plot.add_subplot(1, 1, 1)

            self.line_plot, = self.ax_plot.plot([], lw=3)

            self.ax_plot.set_xlim(self.x_plot.min(), self.x_plot.max())
            self.ax_plot.set_ylim([0, 100000])

            # note that the first draw comes before setting data
            self.fig_plot.canvas.draw()

            self.ax2background = self.fig_plot.canvas.copy_from_bbox(
                self.ax_plot.bbox)

            plt.show(block=True)

    def do_plot(self, hist_full):
        """
        Function to plot the histogram
        Params:
            hist_full   (numpy.ndarray) : #One dimensional array. Index of array is the color intensity. The array values are pixels.
        
        """
        self.line_plot.set_data(self.x_plot, hist_full)
        # restore background
        self.fig_plot.canvas.restore_region(self.ax2background)

        # redraw just the points
        self.ax_plot.draw_artist(self.line_plot)

        # fill in the axes rectangle
        self.fig_plot.canvas.blit(self.ax_plot.bbox)

        self.fig_plot.canvas.flush_events()

    def ocfa_cb(self, msg, threshold_o=11000):
        """
          ***Callback***
        Publishes a boolean value if the amount of orange pixels (100% red, 60% green) is higher than the threshhold.
        
        Params:
            msg (sensor_msgs::Image) : The message in the topic callback.
            threshold_o (int)   : [Default: 30000] Threshold for the frequency detection.
        """
        cv_image = self.bridge_to_cv(msg)

        new_cfd = cv_image.copy()

        # Calculates a histogram of a set of arrays
        hist_red = cv.calcHist([new_cfd], [2], None, [256], [0, 255])
        hist_green = cv.calcHist([new_cfd], [1], None, [256], [0, 255])

        # merging two channels to get orange channel instead
        hist_full = (hist_red + hist_green * 0.6) / 1.6

        # Postprocessing histogram to get relative intensity
        index = np.argmax(hist_full)
        max = hist_full[index]
        mean = np.sum(hist_full) / len(hist_full)
        difference = max - mean

        # Publishs and outputs the information if gate is detected or not
        rospy.loginfo(difference)
        if (difference > threshold_o):
            gateDetection = True
            self.ocfa_pub.publish(
                gateDetection)  # Publishes boolean gateDetection
        else:
            gateDetection = False
            self.ocfa_pub.publish(
                gateDetection)  # Publishes boolean gateDetection

        # Plot the histogram
        if (self.bool):
            self.do_plot(hist_full)

    def rcfa_cb(self, msg, threshold_r=30000):
        """
        ***Callback***
        Publishes a boolean value if the amount of red pixels is higher than the threshhold.
        Params:
            msg (sensor_msgs/Image) : The message in the topic callback.
            threshold_r (int)   : [Default: 30000] Threshold for the frequency detection.
        """
        cv_image = self.bridge_to_cv(msg)
        new_cfd = cv_image.copy()

        # Calculates a histogram of a set of arrays
        hist_full = cv.calcHist([new_cfd], [2], None, [256], [0, 255])

        # Postprocessing histogram to get relative intensity
        index = np.argmax(hist_full)
        max = hist_full[index]
        mean = np.sum(hist_full) / len(hist_full)
        difference = max - mean

        # Publishs and outputs the information if gate is detected or not
        if (difference > threshold_r):
            gateDetection = True
            self.rcfa_pub.publish(gateDetection)  #Publishes gateDetection
        else:
            gateDetection = False
            self.rcfa_pub.publish(gateDetection)

        # Plot the histogram
        if (self.bool):
            self.do_plot(hist_full)

    def bridge_to_cv(self, image_msg, encoding="passthrough"):
        """
        This function converts a ros image into a cv image 
        
        Params:
            image_msg: [sensor_msgs::Image message] Input image in ros format
            encoding: [str] (default = passtrough) The encoding of the image data.
        Return:
            image_transformed: [cv::Mat] Output image in OpenCV format
        """

        image_transformed = None
        try:
            image_transformed = self.bridge.imgmsg_to_cv2(image_msg, encoding)
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
        return image_transformed

    def bridge_to_image(self, cv_image_msg, encoding="passthrough"):
        """
        This function converts a cv image into a ros image 
            
        Params:
            cv_image_msg: [cv::Image] Input image in cv format
            encoding: [str] (default = passtrough) The encoding of the image data.
        Return:
            image_transformed: [sensor_msgs::Image message] Output image in ros format
        """

        image_transformed = None
        try:
            image_transformed = self.bridge.cv2_to_imgmsg(
                cv_image_msg, encoding)
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
        return image_transformed


if __name__ == '__main__':
    node = ncfaDetectionNode()

    while not rospy.is_shutdown():
        rospy.spin()
