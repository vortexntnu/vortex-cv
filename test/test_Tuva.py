#!/usr/bin/env python

import cv2
import numpy as np
import matplotlib.pyplot as plt

from feature_detection  import ImageFeatureProcessing as ifp
from image_extraction import Image_extraction as IE

img = cv2.imread("./data/path_bendy_full.png")
img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

#plt.figure(1, figsize=(10, 10))
#plt.imshow(img_rgb)

x = ifp(img)
y =  IE()

hsv_hue_min = 1
hsv_hue_max = 50
hsv_sat_min = 1
hsv_sat_max = 50
hsv_val_min = 1
hsv_val_max = 50

img_hsv_processor, a, b = x.hsv_processor(
    img,
    hsv_hue_min,
    hsv_hue_max,
    hsv_sat_min,
    hsv_sat_max,
    hsv_val_min,
    hsv_val_max)

img_hsv_processor_gr = cv2.cvtColor(img_hsv_processor, cv2.COLOR_BGR2RGB)

#plt.figure(1, figsize=(10,10))
#y.get_histogram(img)

lower_bgr, upper_bgr = y.define_useful_colors(img_hsv_processor)

fitline = y.extract_useful_colors(img_hsv_processor, lower_bgr,upper_bgr)

plt.figure(2, figsize=(10,10))
#y.get_histogram(img_hsv_processor)

plt.imshow(fitline)

print('hello world')

plt.show()