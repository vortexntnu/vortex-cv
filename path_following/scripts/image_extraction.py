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


    def get_histogram(self, image):
        """Gives a colorhistogram of an image."""
        img = cv.imread('###Insert image###')
        color = ('b','g','r')
        for i,col in enumerate(color):
            histr = cv.calcHist([img],[i],None,[256],[0,256])
            plt.plot(histr,color = col)
            plt.xlim([0,256])
        plt.show()


    def apply_HoughlinesP(self,image):
        """Takes a graycale image, and returns an Image with lines. In theory is better for straight lines, worse if lines are inperfect"""

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

    def apply_FitLine(self,image):
        """Takes a grayscale Image, and returns an image with lines"""



