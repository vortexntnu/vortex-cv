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
    upper_bgr = [255, 255, 255]

    def get_histogram(self, img):
        """Gives a colorhistogram of an image."""
        #img = cv.imread(image)
        color = ('b', 'g', 'r')

        for i, col in enumerate(color):
            histr = cv.calcHist([img], [i], None, [256], [0, 256])
            plt.plot(histr, color=col)
            plt.xlim([0, 255])

    def get_HSV_histogram(self, img):
        """Gives a histogram to be used for tuning hue-values."""
        # Convert the image from BGR to HSV color space
        hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

        # Extract the hue channel
        hue = hsv[:, :, 0]

        # Calculate the histogram of hue values
        hist, bins = np.histogram(hue.ravel(), 180, [0, 180])

        # Plot the histogram
        plt.plot(hist)
        plt.xlim([0, 180])
        plt.xlabel('Hue value')
        plt.ylabel('Pixel count')
        plt.show()

    def apply_HoughlinesP(self, image):
        """Takes a grayscale image, and returns an image with lines. In theory is better for straight lines, worse if lines are inperfect"""

        #Outputs an image of edges
        edges = cv.canny(image,
                         threshold1=self.threshold1,
                         threshold2=self.threshold2)

        #Returns a vector of lines(x_start, y_start, x_end, y_end)
        lines = cv.HoughlinesP(edges,
                               rho=self.resolution,
                               theta=self.theta,
                               threshold=self.threshold,
                               minLineLength=self.minLength,
                               maxLineGap=self.maxLineGap)

        #Draw lines on the image
        if lines is not None:
            for i in range(0, len(lines)):
                max_array_length = lines[i][0]
                cv.line(image, (max_array_length[0], max_array_length[1]),
                        (max_array_length[2], max_array_length[3]),
                        (0, 0, 255), 3, cv.LINE_AA)
        return image

    def contours_from_colors(self, image, lower_rgb, upper_rgb):
        """Takes a colored Image, and returns an image with lines. Lower and upper rgb is 1X3-arrays representing the colors rgb to be used. The histogram will help finding values.
        NB! Here is RGB, cv default is BGR"""
        # Convert image to HSV color space
        hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)

        # Threshold the image to get only wanted colors
        mask = cv.inRange(hsv, lower_rgb, upper_rgb)

        # Find the contour of the blue objects in the image
        contours = cv.findContours(mask, cv.RETR_EXTERNAL,
                                   cv.CHAIN_APPROX_SIMPLE)

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
        start_point = (int(vector[2]), int(vector[3]))
        end_point = (int(vector[2]) + int(vector[0] * 10000),
                     int(vector[3]) + int(vector[1] * 10000))
        line_image = cv.line(img, start_point, end_point, (0, 0, 255), 4)

        return line_image

    def YellowEdgesHSV(self, img, lower_yellow, upper_yellow):
        """Takes in an cv-image, lower_yellow threshold and upper level threshold."""

        # Convert the image to HSV color space
        hsv_img = cv.cvtColor(img, cv.COLOR_BGR2HSV)

        # Threshold the hue channel to extract yellow color pixels
        yellow_hue_range_low = np.array([lower_yellow, 100, 100])
        yellow_hue_range_high = np.array([upper_yellow, 255, 255])
        yellow_mask = cv.inRange(hsv_img, yellow_hue_range_low,
                                 yellow_hue_range_high)

        # Apply the mask to the hue channel to focus on yellow colors
        hue_channel = cv.bitwise_and(hsv_img[:, :, 0],
                                     hsv_img[:, :, 0],
                                     mask=yellow_mask)

        # Compute the gradient of the hue channel
        sobelx = cv.Sobel(hue_channel, cv.CV_64F, 1, 0, ksize=5)
        sobely = cv.Sobel(hue_channel, cv.CV_64F, 0, 1, ksize=5)
        grad = cv.magnitude(sobelx, sobely)

        # Apply thresholding to highlight the edges
        thresh = cv.threshold(grad, 50, 255, cv.THRESH_BINARY)[1]
        return thresh

    def EdgesHSV_adapting(self, img):
        # Convert the image to HSV color space
        hsv_img = cv.cvtColor(img, cv.COLOR_BGR2HSV)

        # Extract the hue channel from the HSV image
        hue_channel = hsv_img[:, :, 0]

        # Compute the gradient of the hue channel
        sobelx = cv.Sobel(hue_channel, cv.CV_64F, 1, 0, ksize=5)
        sobely = cv.Sobel(hue_channel, cv.CV_64F, 0, 1, ksize=5)
        grad = cv.magnitude(sobelx, sobely)

        # Apply thresholding to highlight the edges
        thresh = cv.threshold(grad, 50, 255, cv.THRESH_BINARY)[1]
        return thresh

    def onlyYellow(self, img, lower_yellow, upper_yellow):
        # Convert the image to HSV color space
        hsv_img = cv.cvtColor(img, cv.COLOR_BGR2HSV)

        # Threshold the hue channel to extract yellow color pixels
        yellow_hue_range_low = np.array([lower_yellow, 100, 100])
        yellow_hue_range_high = np.array([upper_yellow, 255, 255])
        yellow_mask = cv.inRange(hsv_img, yellow_hue_range_low,
                                 yellow_hue_range_high)

        # Apply the mask to the hue channel to focus on yellow colors
        hue_channel = cv.bitwise_and(hsv_img[:, :, 0],
                                     hsv_img[:, :, 0],
                                     mask=yellow_mask)

        # Apply thresholding to highlight the edges
        thresh = cv.threshold(hue_channel, 50, 255, cv.THRESH_BINARY)[1]
        return thresh
