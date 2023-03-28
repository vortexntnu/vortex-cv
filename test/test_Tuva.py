#!/usr/bin/env python

import cv2
import numpy as np
import matplotlib.pyplot as plt

from feature_detection import ImageFeatureProcessing as ifp
from image_extraction import Image_extraction as IE
"""
#images
"./data/yellowcylinder.jpeg"
"./data/yellowsubandmore.jpg"
"./data/yellowsubmarine.jpeg"
"./data/boaty.jpeg"
"./data/differentcoloredpipe.jpeg"
"./data/diffusePipe.jpeg"
"./data/fishy.jpeg"
"./data/noisy.jpeg"
"./data/path_bendy_full.png"
"./data/thindiffusepipe.jpeg"
"./data/veryclean.jpeg"
"./data/yellowcylinder.jpeg"
"./data/dockingst.png"
"""

img = cv2.imread("./data/dockingst.png")
img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
img_gr = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

#class instances to access feature detection and image extraction
x = ifp(img)
y = IE()

#Tuning parameters

hsv_hue_min = 60
hsv_hue_max = 80
hsv_sat_min = 1
hsv_sat_max = 50
hsv_val_min = 1
hsv_val_max = 50

img_hsv_processor, a, b = x.hsv_processor(img, hsv_hue_min, hsv_hue_max,
                                          hsv_sat_min, hsv_sat_max,
                                          hsv_val_min, hsv_val_max)

#Hue-limits
lower_yellow = 20
upper_yellow = 60

#Plotting edges found with
yellow_img = y.YellowEdgesHSV(img, lower_yellow, upper_yellow)
plt.figure("HSV_adaption", figsize=(10, 10))
plt.imshow(yellow_img)

only_img = y.onlyYellow(img, lower_yellow, upper_yellow)
plt.figure("Only_yellow", figsize=(10, 10))
plt.imshow(only_img)

plt.figure("Histogram", figsize=(10, 10))
y.get_HSV_histogram(img)

plt.show()
