import numpy as np
import matplotlib.pyplot as plt

import cv2 as cv


def rgb_to_gray(I):
    """
    Converts a HxWx3 RGB image to a HxW grayscale image.

    """
    R = I[:, :, 0]
    G = I[:, :, 1]
    B = I[:, :, 2]
    return (R + G + B) / 3.0


def gaussian(I, sigma):
    """
    Applies a 2-D Gaussian blur with standard deviation sigma to
    a grayscale image I.
    """

    # Generate the 1-D Gaussian filter kernel
    h = int(np.ceil(3 * sigma))
    x = np.linspace(-h, +h, 2 * h + 1)
    g = np.exp(-x**2 / (2 * sigma**2)) / np.sqrt(2 * np.pi * sigma**2)

    # Filter the image (using the fact that the Gaussian is separable)
    Ig = np.zeros_like(I)
    for row in range(I.shape[0]):
        Ig[row, :] = np.convolve(I[row, :], g, mode='same')
    for col in range(I.shape[1]):
        Ig[:, col] = np.convolve(Ig[:, col], g, mode='same')
    return Ig


threshold = 0.02
sigma = 5
filename = '/home/beng/projects/vortex/cv_ws/src/Vortex-CV/sift_feature_detection/data/sift_images/valve_hd/cropped_valve2.png'

I_rgb = cv.imread(filename)
I_rgb = I_rgb / 255.0
I_gray = rgb_to_gray(I_rgb)
I_blur = gaussian(I_gray, sigma)

I_blur = I_blur * 255

cv.imwrite('/home/beng/projects/vortex/cv_ws/src/Vortex-CV/sift_feature_detection/data/sift_images/valve/cropped_valve2_blur.png', I_blur)
# plt.imshow(I_blur, cmap="gray")
# # plt.savefig("g-man_blur.png")
# plt.savefig('/home/beng/projects/vortex/cv_ws/src/Vortex-CV/sift_feature_detection/data/sift_images/valve/cropped_valve_blur.png',
#             bbox_inches='tight',
#             transparent=True,
#             pad_inches=0)  #TODO Needs to remove axis as well
# plt.show
