
import cv2
import numpy as np
import matplotlib.pyplot as plt

from feature_detection  import ImageFeatureProcessing as ifp

"""img = cv2.imread("./data/path_bendy_full.png")
img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

plt.figure(1, figsize=(10, 10))
plt.imshow(img_rgb)

x = ifp(img)

hsv_hue_min = 1
hsv_hue_max = 50
hsv_sat_min = 1
hsv_sat_max = 50
hsv_val_min = 1
hsv_val_max = 50

img2, a, b = x.hsv_processor(
    img,
    hsv_hue_min,
    hsv_hue_max,
    hsv_sat_min,
    hsv_sat_max,
    hsv_val_min,
    hsv_val_max)

img2_rgb = cv2.cvtColor(img2, cv2.COLOR_BGR2RGB)

plt.figure(2, figsize=(10, 10))
plt.imshow(img2_rgb)

print('hello world')

plt.show()"""
plt.show()