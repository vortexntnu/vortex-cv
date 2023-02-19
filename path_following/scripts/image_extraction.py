#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import cv2 as cv


class image_extraction():
    """Extracting various useful data from images. Class made to compare different methods of approach.
    Preprocessing can be done with functions from ImageFeatureProcessing"""
    
    #Default values for variuos functions, not recommended
    # Houghlines  
    threshold1 = 100
    threshold2 = 200

    resolution = 1
    theta = np.pi / 180
    threshold = 50
    minlength = 200
    maxLineGap = 40

    #Colours
    min_intensity = 150
    upper_bgr = [255,255,255]


    def get_histogram(self, image):
        """Gives a colorhistogram of an image."""
        img = cv.imread(image)
        color = ('b','g','r')

        for i,col in enumerate(color):
            histr = cv.calcHist([img],[i],None,[256],[0,256])
            plt.plot(histr,color = col)
            plt.xlim([0,256])
        plt.show()

    def define_useful_colors(self,image):
        """In development. Takes colored image, returns threshold values as a bgr-array.
        To be used when in new,unfamiliar enviroment, and looking at known object, filling more than 1/2 of object. Returns the interval of most detected rgb-intensity.
        The hope is that we now can use colorextraction to track an object under water."""
        img = cv.imread(image)

        #hist is an array containing number of appearings for the different colorintensity, 256x3 array
        hist_b = cv.calcHist(img,[0], 256, [0,255])
        hist_g = cv.calcHist(img,[1], 256, [0,255])
        hist_r = cv.calcHist(img,[2], 256, [0,255])

        threshold_num_constant = 0.7

        """
        #Expecting problem with 0.7* value doesnt perfectly match list-value 
        
        lower_b = hist_b.index(max(hist_b)*threshold_num_constant)
        upper_b = hist_b[lower_b+1,:].index(max(hist_b)*threshold_num_constant)

        lower_g = hist_g.index(max(hist_g)*threshold_num_constant)
        upper_g = hist_g[lower_g+1,:].index(max(hist_g)*threshold_num_constant)

        lower_r = hist_r.index(max(hist_r)*threshold_num_constant)
        upper_r = hist_r[lower_r+1,:].index(max(hist_r)*threshold_num_constant)

        Should manage this method, the one below relies on intensity-distance from max-appearance, this one on number of apperance compared to max apperances. 
        """

        lower_b = int(max(hist_b) * threshold_num_constant)
        upper_b = int(max(hist_b)* (1 + (1-threshold_num_constant)))

        lower_g = int(max(hist_g) * threshold_num_constant)
        upper_g = int(max(hist_g)* (1 + (1-threshold_num_constant)))

        lower_r = int(max(hist_r) * threshold_num_constant)
        upper_r = int(max(hist_r)* (1 + (1-threshold_num_constant)))

        lower_bgr = [lower_b,lower_g,lower_r]
        upper_bgr = [upper_b,upper_g,upper_r]

        return lower_bgr, upper_bgr

    
    def extract_useful_colors(self,image, lower_bgr, upper_bgr):
        """In development. Takes a lower_bgr from define_useful_colors. Desired function: Returns a binary image with pixels = 1 for those with correct color. 
        Note, the define-color functions has to be used once to get the threshold, which can be used multiple times in this function."""

        img = cv.imread(image)
        #Computes binary image, with ones for pixels between lower and upper bgr
        valid_colors_img = cv.threshold(img, lower_bgr, upper_bgr, cv.THRESH_BINARY)

        points = np.where(valid_colors_img > 0)

        #pathlines is a tuple of (vx, vy, x and y), which represents a vector x y and a point of origin. 
        path_line = cv.fitline(points, cv.DIST_L1)

        return path_line

    def apply_HoughlinesP(self,image):
        """Takes a grayscale image, and returns an image with lines. In theory is better for straight lines, worse if lines are inperfect"""

        #Outputs an image of edges
        edges = cv.canny(image, threshold1 = self.threshold1, threshold2 = self.threshold2)

        #Returns a vector of lines(x_start, y_start, x_end, y_end)
        lines = cv.HoughlinesP(edges, rho = self.resolution, theta= self.theta, threshold = self.threshold, minLineLength = self.minLength, maxLineGap = self.maxLineGap)

        #Draw lines on the image
        if lines is not None:
            for i in range(0, len(lines)):
                l = lines[i][0]
                cv.line(image,(l[0], l[1]), (l[2], l[3]), (0,0,255), 3, cv.LINE_AA)
        return image

    def contours_from_colors(self,image,lower_rgb, upper_rgb):
        """Takes a colored Image, and returns an image with lines. Lower and upper rgb is 1X3-arrays representing the colors rgb to be used. The histogram will help finding values.
        NB! Here is RGB, cv2 default is BGR"""
        # Convert image to HSV color space
        hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)

        # Threshold the image to get only wanted colors
        mask = cv.inRange(hsv, lower_rgb, upper_rgb)

        # Find the contour of the blue objects in the image
        contours = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        # Extract the points within the blue color range
        points = []
        for contour in contours:
            for point in contour:
                points.append(point[0])

        # Draw the contours on the original image
        cv.drawContours(image, contours, -1, (0, 255, 0), 3)



