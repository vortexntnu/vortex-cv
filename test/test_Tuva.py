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

plt.figure(1)

y.get_histogram(img_hsv_processor)

lower_bgr, upper_bgr = y.define_useful_colors(img_hsv_processor)

plt.figure(3)
fitLine = y.extract_useful_colors(img_hsv_processor, lower_bgr, upper_bgr)
line_image = y.drawline(img_hsv_processor, fitLine)

#Vektor gaar feil vei ass.
plt.figure(2)
plt.imshow(line_image)


lower_rgb = np.array([lower_bgr[2], lower_bgr[1], lower_bgr[0]])
upper_rgb = np.array([upper_bgr[2], upper_bgr[1], upper_bgr[0]])

print('hello world')
plt.show()