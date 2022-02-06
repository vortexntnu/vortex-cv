from tkinter.messagebox import NO
import numpy as np
import cv2 as cv
import glob
import os
from PIL import Image
from sklearn.feature_extraction import image
import time

path_left = "/home/vortex/vortex_ws/src/Vortex_CV/camera_calibration/calibrationdata/calibration_data_left/left-0040.png"
path_right = "/home/vortex/vortex_ws/src/Vortex_CV/camera_calibration/calibrationdata/calibration_data_right/right-0040.png"
# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

objp = np.zeros((6*8,3), np.float32)
objp[:,:2] = np.mgrid[0:8,0:6].T.reshape(-1,2)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objpoints = [] # 3d point in real world space
imgpoints_left = [] # 2d points in image plane.
imgpoints_right = [] # 2d points in image plane.

# Arrays to store object points and image points from all the images.
imgpoints_left = [] # 2d points in image plane.
imgpoints_right = [] # 2d points in image plane.

img_left = cv.imread(path_left)
img_right = cv.imread(path_right)
gray_left = cv.cvtColor(img_left, cv.COLOR_BGR2GRAY)
gray_right = cv.cvtColor(img_right, cv.COLOR_BGR2GRAY)

ret, corners = cv.findChessboardCorners(img_left, (8,6), None)
if ret:
    print("true")
imgpoints_left.append(corners)
objpoints.append(objp)

ret, corners = cv.findChessboardCorners(img_right, (8,6), None)
if ret:
    print("true")
imgpoints_right.append(corners)
#objpoints.append(objp)

retL, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints_left, gray_left.shape[::-1], None, None)
retR, mtx2, dist2, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints_right, gray_right.shape[::-1], None, None)


ret, mtx, dist, mtx2, dist2, R, T, E, F = cv.stereoCalibrate(objpoints, imgpoints_left, imgpoints_right, mtx, dist, mtx2, dist2, gray_left.shape[::-1], None, None, None, None)

print("ret: ", ret)
print("mtx: ", mtx)
print("dist 1: ", dist)
print("mtx2: ", mtx2)
print("dist2: ", dist2)
print("R: ", R)
print("T: ", T)
print("E: ", E)
print("F: ", F)


