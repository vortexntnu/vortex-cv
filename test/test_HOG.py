import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
from skimage.feature import hog

img = cv.imread("./data/path_bendy_full.png")

gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

hog_features, hog_image = hog(gray, orientations=8, pixels_per_cell=(16, 16),
                              cells_per_block=(1, 1), block_norm='L2-Hys', visualize=True)

plt.figure(1, figsize=(10, 10))
plt.imshow(hog_image)
plt.show()

print(type(hog_image))