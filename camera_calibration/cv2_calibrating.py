from tkinter.messagebox import NO
import numpy as np
import cv2 as cv
import glob
import os
from PIL import Image
from sklearn.feature_extraction import image

path = "/home/vortex/cv_ws/src/Vortex_CV/camera_calibration/calibrationdata/calibration_data_left/"

# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*8,3), np.float32)
objp[:,:2] = np.mgrid[0:8,0:6].T.reshape(-1,2)
# Arrays to store object points and image points from all the images.
objpoints_left = [] # 3d point in real world space
imgpoints_left = [] # 2d points in image plane.
objpoints_right = [] # 3d point in real world space
imgpoints_right = [] # 2d points in image plane.



directory = os.fsencode(path)

ret_list_left = []
mtx_list_left = []
dist_list_left = []
rvec_list_left = []
tvec_list_left = []


ret_list_right = []
mtx_list_right = []
dist_list_right = []
rvec_list_right = []
tvec_list_right = []


for file in sorted(os.listdir(directory)):
    filename = os.fsdecode(file)
    if filename.endswith(".png"):
        filename = path + filename
        print(filename)

        img = cv.imread(filename)
        #gray = cv.imread(filename)
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

        image = Image.open(filename)
        image_array = np.array(image)
        print(image_array.shape)
        #cv.imshow("img", img)
        # Find the chess board corners
        ret, corners = cv.findChessboardCorners(img, (8,6), None)
        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints_left.append(objp)
            #corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
            imgpoints_left.append(corners)
            # Draw and display the corners
            cv.drawChessboardCorners(img, (8,6), corners, ret)
            # cv.imshow('img', img)
            # cv.waitKey(500)
            # Calibration
            ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints_left, imgpoints_left, gray.shape[::-1], None, None)

            # Undistortion
            h, w = img.shape[:2]
            new_camera_mtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
            dst = cv.undistort(img, mtx, dist, None, new_camera_mtx)

            # Crop the image
            x, y, w, h = roi
            dst = dst[y:y+h, x:x+h]
            # cv.imshow("calibrate_result", dst)
            # cv.waitKey(500)

            ret_list_left.append(ret)
            mtx_list_left.append(mtx)
            dist_list_left.append(dist)
            rvec_list_left.append(rvecs)
            tvec_list_left.append(tvecs)

            # print("ret: " + str(ret))
            # print("camera matrix: ", mtx)
            # print("dist: ", dist)

path = "/home/vortex/cv_ws/src/Vortex_CV/camera_calibration/calibrationdata/calibration_data_right/"

directory = os.fsencode(path)


for file in sorted(os.listdir(directory)):
    filename = os.fsdecode(file)
    if filename.endswith(".png"):
        filename = path + filename
        print(filename)

        img = cv.imread(filename)
        #gray = cv.imread(filename)
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

        image = Image.open(filename)
        image_array = np.array(image)
        print(image_array.shape)
        #cv.imshow("img", img)
        # Find the chess board corners
        ret, corners = cv.findChessboardCorners(img, (8,6), None)
        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints_right.append(objp)
            #corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
            imgpoints_right.append(corners)
            # Draw and display the corners
            cv.drawChessboardCorners(img, (8,6), corners, ret)
            # cv.imshow('img', img)
            # cv.waitKey(500)
            # Calibration
            ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints_right, imgpoints_right, gray.shape[::-1], None, None)

            # Undistortion
            h, w = img.shape[:2]
            new_camera_mtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
            dst = cv.undistort(img, mtx, dist, None, new_camera_mtx)

            # Crop the image
            x, y, w, h = roi
            dst = dst[y:y+h, x:x+h]
            # cv.imshow("calibrate_result", dst)
            # cv.waitKey(500)

            ret_list_right.append(ret)
            mtx_list_right.append(mtx)
            dist_list_right.append(dist)
            rvec_list_right.append(rvecs)
            tvec_list_right.append(tvecs)

            # print("ret: " + str(ret))
            # print("camera matrix: ", mtx)
            # print("dist: ", dist)

# mean_error = 0
# for i in range(len(objpoints)):
#     imgpoints2, _ = cv.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
#     error = cv.norm(imgpoints[i], imgpoints2, cv.NORM_L2)/len(imgpoints2)
#     mean_error += error
# print( "total error: {}".format(mean_error/len(objpoints)) )



            

cv.CALIB_FIX_INTRINSIC

cv.destroyAllWindows()