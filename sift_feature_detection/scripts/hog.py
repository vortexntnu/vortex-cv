from skimage.io import imread, imshow
from skimage.transform import resize
from skimage.feature import hog
from skimage import exposure
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image

import cv2 as cv

resized_img = imread('../data/sift_images/gate.png')

# resized_img = imread('../data/sift_images/huseby_gate_far.png')
# print(resized_img.shape)


# converting image into grayscale image
gray = cv.cvtColor(resized_img, cv.COLOR_BGR2GRAY)
  
# setting threshold of gray image
_, threshold = cv.threshold(gray, 127, 255, cv.THRESH_BINARY)

#imshow(img)
#print(img.shape)
# resized_img = resize(resized_img, (64, 128)) 
#imshow(resized_img) 
#print(resized_img.shape)
fd, hog_image = hog(resized_img, orientations=9, pixels_per_cell=(8, 8), 
                    cells_per_block=(2, 2), block_norm = "L2-Hys", visualize=True, multichannel=True)

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8), sharex=True, sharey=True) 

ax1.imshow(resized_img, cmap=plt.cm.gray) 
ax1.set_title('Input image') 

# Rescale histogram for better display 
hog_image_rescaled = exposure.rescale_intensity(hog_image, in_range=(0, 10)) 

img = np.copy(hog_image_rescaled)


ax2.imshow(hog_image_rescaled, cmap=plt.cm.gray) 
ax2.set_title('Histogram of Oriented Gradients')

plt.show()


im = np.array(hog_image_rescaled, dtype = np.uint8)


gate_shape = imread('../data/sift_images/gate_shape.png')


ret, threshed  = cv.threshold(img,0.4,255,cv.THRESH_BINARY)

# using a findContours() function
# contours, _ = cv.findContours(threshed, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
  
# i = 0
  
# # list for storing names of shapes
# for contour in contours:
  
#     # here we are ignoring first counter because 
#     # findcontour function detects whole image as shape
#     if i == 0:
#         i = 1
#         continue
  
#     # cv2.approxPloyDP() function to approximate the shape
#     approx = cv.approxPolyDP(
#         contour, 0.01 * cv.arcLength(contour, True), True)
      
#     # using drawContours() function
#     cv.drawContours(resized_img, [contour], 0, (0, 0, 255), 5)
  
#     # finding center point of shape
#     M = cv.moments(contour)
#     if M['m00'] != 0.0:
#         x = int(M['m10']/M['m00'])
#         y = int(M['m01']/M['m00'])


cv.imshow("hog", threshed)
cv.waitKey(0) 