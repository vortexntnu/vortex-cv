#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import cv2 as cv


class Image_extraction():
    """Extracting various useful data from images. Class made to compare different methods of approach.
    Preprocessing can be done with functions from ImageFeatureProcessing"""

    def __init__(self):
        pass
    
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


    def get_histogram(self, img):
        """Gives a colorhistogram of an image."""
        #img = cv.imread(image)
        color = ('b','g','r')

        for i,col in enumerate(color):
            histr = cv.calcHist([img],[i],None,[256],[0,256])
            plt.plot(histr,color = col)
            plt.xlim([0,255])
        

    def define_useful_colors(self,img):
        """In development. Takes colored image, returns threshold values as a bgr-array.
        To be used when in new,unfamiliar enviroment, and looking at known object, filling more than 1/2 of object. Returns the interval of most detected rgb-intensity.
        The hope is that we now can use colorextraction to track an object under water."""
        ##img = cv.imread(image)

        #hist is an array containing number of appearings for the different colorintensity, 256x3 array
        hist_b = cv.calcHist([img],[0], None, [256], [0,256])
        hist_g = cv.calcHist([img],[1], None, [256], [0,256])
        hist_r = cv.calcHist([img],[2], None, [256], [0,256])

        """ 
        Trying to compute integrals using riemannsums: (Need to include index)
        L = 256 
        N = 1000
        h = L/N

        ##min_intensity_integral = 5000

        sum_b = 0
        for i in range(N):
            sum_b += 0.5 * (hist_b[n*h] + hist_b[(n+1)*h])
        sum_b = sum_b * h 
         """

        
        #Finding indexes with colors above percentage of maxima,percentage given by threshold_intensity_constant
        threshold_intensity_constant =  0.01
        L = len(hist_b)
        
        b_lower = 0
        b_upper = 0

        b_max = max(hist_b)
        b_threshold =  b_max * threshold_intensity_constant

        max_array_length = 0
        i = 0
        for i in range(L):
            if hist_b[i] > b_threshold:
                count = 0
                while(hist_b[i + count] > b_threshold):
                    count += 1
                if count > max_array_length:
                    max_array_length = count 
                    b_lower = i

                    b_upper = i + max_array_length
                i += count  
            
        
        max_array_length = 0
        for i in range(L):
            
            if hist_g[i] > max(hist_g) * threshold_intensity_constant:
                count = 0

                while(hist_g[i + count] > max(hist_g) * threshold_intensity_constant):
                    count += 1

                if count > max_array_length:
                    max_array_length = count 
                    g_lower = i

                    g_upper = i + max_array_length
                i += count  

        max_array_length = 0
        for i in range(L):

            if hist_r[i] > max(hist_r) * threshold_intensity_constant:
                count = 0
                while(hist_r[i + count] > max(hist_r) * threshold_intensity_constant):
                    count += 1
                if count > max_array_length:
                    max_array_length = count 
                    r_lower = i

                    r_upper = i + max_array_length
                i += count  
        

        lower_bgr = np.array([b_lower, g_lower, r_lower])
        upper_bgr = np.array([b_upper,g_upper,r_upper])
        
        
        return lower_bgr, upper_bgr


    
    def extract_useful_colors(self,img, lower_bgr, upper_bgr):
        """In development. Takes a lower_bgr from define_useful_colors. Desired function: Returns a binary image with pixels = 1 for those with correct color. 
        Note, the define-color functions has to be used once to get the threshold, which can be used multiple times in this function."""

        #img = cv.imread(image)
        #Computes binary image, with ones for pixels between lower and upper bgr
        valid_color_image = cv.inRange(img, lower_bgr, upper_bgr)

        plt.imshow(valid_color_image)

        points  = np.array(cv.findNonZero(valid_color_image))

        #pathlines is a tuple of (vx, vy, x and y), which represents a vector x y and a point of origin. 
        path_line = cv.fitLine(points, cv.DIST_L2,0, 0.01,0.01)

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
                max_array_length = lines[i][0]
                cv.line(image,(max_array_length[0], max_array_length[1]), (max_array_length[2], max_array_length[3]), (0,0,255), 3, cv.LINE_AA)
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
        """
        points = []
        for contour in contours:
            for point in contour:
                points.append(point[0])
        """
        # Draw the contours on the original image
        cv.drawContours(image, contours, -1, (0, 255, 0), 3)

    def drawline(self, img, vector):
        """Made to draw the line from fitLine."""
        start_point = (int(vector[2]),int(vector[3]))
        end_point  = (  int(vector[2]) + int(vector[0]*10000)   ,   int(vector[3]) + int(vector[1]*10000)  ) 
        line_image = cv.line(img,start_point,end_point, (0,0,255), 4)

        return line_image



