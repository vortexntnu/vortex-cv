import cv2
import matplotlib.pyplot as plt
import numpy as np

import cv2 as cv
from HOG import HOG


def threshold(hog_img):
    '''
    threshold = 0.5 * hog_img.max()
    if threshold < 2:
        threshold = 2
    binary_image = hog_img > threshold
    '''

    img_norm = cv2.normalize(hog_img,
                             None,
                             alpha=0,
                             beta=255,
                             norm_type=cv2.NORM_MINMAX,
                             dtype=cv2.CV_8U)

    # Apply Otsu's method to calculate the threshold value
    ret, threshold = cv2.threshold(img_norm, 0, 255,
                                   cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    binary_image = np.array(threshold, dtype=np.float32) / 255

    return binary_image


img = cv.imread("./images/image_23.jpg")
extractor = HOG((10, 10), (1, 1), 3)
hog_descriptor, hog_img = extractor.compute_hog(img)
binary_img = threshold(hog_img)

plt.figure(1, figsize=(10, 10))
plt.imshow(hog_img)

plt.figure(2, figsize=(10, 10))
plt.imshow(binary_img)
plt.show()
