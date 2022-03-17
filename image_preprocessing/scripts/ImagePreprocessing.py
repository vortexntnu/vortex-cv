#! /usr/bin/env python
import cv2 as cv
import numpy as np
from scipy import ndimage

class ImagePreprocessing:

    """
    config is an array of True/False values which determine how the image will be pre-processed
    config = [BGR,  CLAHE,   GAUSSIAN_BLUR,   GARY_WORLD,  SVD_COMPRESSION,  ]

    params = [channel, clahe_cliplim, clahe_gridsize, gaussian_sigma, ]

    """

    def __init__(self,
                clahe_cliplim,
                clahe_tilesize
                ):

            self.clahe = cv.createCLAHE(clipLimit=clahe_cliplim, tileGridSize=(clahe_tilesize,clahe_tilesize))


    def CLAHE(self, img):
        equ = np.zeros(np.shape(img))
        
        if len(np.shape(img)) == 2: # If it is a grayscale/one channel image
            equ = self.clahe.apply(img)
            return equ
        elif len(np.shape(img)) < 2:
            raise IndexError("Image must be at least a 2D array")
        else:
            for i in range(np.shape(img)[2]):
                equ[:,:,i] = self.clahe.apply(img[:,:,i])
            return equ

    def SVD_compression(self, img, r):
        U, S, VT = np.linalg.svd(img)
        return np.matmul(np.matmul(U[:,:r], np.diag(S)[:r, :r]), VT [:r, :])
    
    def central_difference(self, I, diff):
        """
        Computes the gradient in the x and y direction using
        a central difference filter, and returns the resulting
        gradient images (Ix, Iy) and the gradient magnitude Im.
        """
        kernel = np.array([-diff, 0, diff])
        Ix = ndimage.convolve1d(I, kernel, axis=1)
        Iy = ndimage.convolve1d(I, kernel, axis=0)
        # TODO: Implement without convolution functions as an exercise
        Im = np.sqrt(Ix ** 2 + Iy ** 2)
        return Ix, Iy, Im

    def gamma_correction(self, img, alpha, beta, gamma, ch=None, benG_single=None):

        look_up = np.empty((1,256), np.uint8)
        for i in range(256):
            look_up[0,i] = np.clip(pow(i / 255.0, gamma) * 255.0, 0, 255)


        if ch != None:
            new_img = (img[:,:,0] * alpha + np.full_like(img[:,:,0], beta)).astype(np.uint8)
            new_img = cv.LUT(new_img, look_up)
            if benG_single:
                return new_img.astype(float)
            else:
                img[:,:,ch] = new_img.astype(float)
                return img
        else:
            new_img = (img * alpha + np.full_like(img, beta)).astype(np.uint8)
            new_img = cv.LUT(new_img, look_up).astype(float)
            return new_img

    def gaussian_filter(self, img, sigma, ch=None):

        K = int(round(2*np.pi*sigma) + 1)
        if K % 2 == 0:
            K += 1
        else:
            K = K
        if ch != None:
            return cv.GaussianBlur(img[:,:,ch], (K,K), sigma)
        else:
            return cv.GaussianBlur(img, (K,K), sigma)
    
    def gray_world(self, img):

        # 1) Calculate illuminant for each channel:     illu_channel = sum_channel / (nr_rows * nr_cols) (This is mean intensity of channel)
        # 2) Scale = (illum_red + illum_greed + illum_blue) / 3
        # 3) Correct each channel according to:     Img_chanel = Img_channel * scale / illuminant_channel (This is Ix * m)
        N = np.shape(img)[0]*np.shape(img)[1]
        img_new = np.empty_like(img)
        illum_red, illum_green, illum_blue = np.sum(img[:,:,2]) / N , np.sum(img[:,:,1]) / N, np.sum(img[:,:,0]) / N
        scale = (illum_red + illum_green + illum_blue) / 3

        img_new[:,:,0] = img[:,:,0] * scale / illum_blue
        img_new[:,:,1] = img[:,:,1] * scale / illum_green
        img_new[:,:,2] = img[:,:,2] * scale / illum_red
        return img_new

    