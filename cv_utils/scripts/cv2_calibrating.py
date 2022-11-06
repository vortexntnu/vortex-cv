from tkinter.messagebox import NO
import numpy as np
import cv2 as cv
import glob
import os
from PIL import Image
from sklearn.feature_extraction import image
import time

path_left = "/home/vortex/calib_ws_2/calibrationdata/calibration_data_left/"
path_right = "/home/vortex/calib_ws_2/calibrationdata/calibration_data_right/"


def calib_cam(path):
    # termination criteria
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((6 * 8, 3), np.float32)
    objp[:, :2] = np.mgrid[0:8, 0:6].T.reshape(-1, 2)
    # Arrays to store object points and image points from all the images.
    objpoints_left = []  # 3d point in real world space
    imgpoints_left = []  # 2d points in image plane.
    objpoints_right = []  # 3d point in real world space
    imgpoints_right = []  # 2d points in image plane.

    directory = os.fsencode(path)

    old_total_error = 10
    for file in sorted(os.listdir(directory)):
        filename = os.fsdecode(file)
        if filename.endswith(".png"):
            filename = path + filename
            print(filename)

            img = cv.imread(filename)
            #gray = cv.imread(filename)
            gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

            # Find the chess board corners
            ret, corners = cv.findChessboardCorners(img, (8, 6), None)
            # If found, add object points, image points (after refining them)
            if ret == True:
                objpoints_left.append(objp)
                #corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
                imgpoints_left.append(corners)

                # Draw and display the corners
                cv.drawChessboardCorners(img, (8, 6), corners, ret)

                ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(
                    objpoints_left, imgpoints_left, gray.shape[::-1], None,
                    None)
                # print((ret))

        mean_error = 0
        for i in range(len(objpoints_left)):
            imgpoints2, _ = cv.projectPoints(objpoints_left[i], rvecs[i],
                                             tvecs[i], mtx, dist)
            error = cv.norm(imgpoints_left[i], imgpoints2,
                            cv.NORM_L2) / len(imgpoints2)
            mean_error += error

        total_error = mean_error / len(objpoints_left)

        if (old_total_error > total_error):
            old_total_error = total_error
            min_err_ret = ret
            # ret_list_left[0]=(ret)
            #print(ret_list_left)
            min_err_mtx = mtx
            # mtx_list_left[0]=(mtx)
            # print(mtx_list_left)
            # dist_list_left[0]=(dist)
            min_err_dist = dist
            # print(dist_list_left)
            # rvec_list_left[0]=(rvecs)
            min_err_rvecs = rvecs
            # print(rvec_list_left)
            min_err_tvecs = tvecs
            min_err_idx = i
            # tvec_list_left[0]=(tvecs)
            # print(tvec_list_left)

            print(min_err_ret, min_err_mtx, min_err_dist, min_err_rvecs,
                  min_err_tvecs, min_err_idx)

    return (min_err_ret, min_err_mtx, min_err_dist, min_err_rvecs,
            min_err_tvecs, min_err_idx)
    # print( "total error: {}".format(mean_error/len(objpoints_left)) )
    #index = total_error_list_left.index(min(total_error_list_left))
    #print("The smallest error is: ", min(total_error_list_left), ". Located at ", total_error_list_left.index(min(total_error_list_left)) )


def stereo_calib_cam(path_left, path_right):
    # termination criteria
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((6 * 8, 3), np.float32)
    objp[:, :2] = np.mgrid[0:8, 0:6].T.reshape(-1, 2)
    # Arrays to store object points and image points from all the images.
    objpoints_left = []  # 3d point in real world space
    imgpoints_left = []  # 2d points in image plane.
    objpoints_right = []  # 3d point in real world space
    imgpoints_right = []  # 2d points in image plane.

    directory_left = os.fsencode(path_left)
    directory_right = os.fsencode(path_right)

    old_total_error = 10
    for file in sorted(os.listdir(directory)):
        filename = os.fsdecode(file)
        if filename.endswith(".png"):
            filename = path + filename
            print(filename)

            img = cv.imread(filename)
            #gray = cv.imread(filename)
            gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

            # Find the chess board corners
            ret, corners = cv.findChessboardCorners(img, (8, 6), None)
            # If found, add object points, image points (after refining them)
            if ret == True:
                objpoints_left.append(objp)
                #corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
                imgpoints_left.append(corners)

                # Draw and display the corners
                cv.drawChessboardCorners(img, (8, 6), corners, ret)

                ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(
                    objpoints_left, imgpoints_left, gray.shape[::-1], None,
                    None)
                # print((ret))

        mean_error = 0
        for i in range(len(objpoints_left)):
            imgpoints2, _ = cv.projectPoints(objpoints_left[i], rvecs[i],
                                             tvecs[i], mtx, dist)
            error = cv.norm(imgpoints_left[i], imgpoints2,
                            cv.NORM_L2) / len(imgpoints2)
            mean_error += error

        total_error = mean_error / len(objpoints_left)

        if (old_total_error > total_error):
            old_total_error = total_error
            min_err_ret = ret
            # ret_list_left[0]=(ret)
            #print(ret_list_left)
            min_err_mtx = mtx
            # mtx_list_left[0]=(mtx)
            # print(mtx_list_left)
            # dist_list_left[0]=(dist)
            min_err_dist = dist
            # print(dist_list_left)
            # rvec_list_left[0]=(rvecs)
            min_err_rvecs = rvecs
            # print(rvec_list_left)
            min_err_tvecs = tvecs
            min_err_idx = i
            # tvec_list_left[0]=(tvecs)
            # print(tvec_list_left)

            print(min_err_ret, min_err_mtx, min_err_dist, min_err_rvecs,
                  min_err_tvecs, min_err_idx)

    return (min_err_ret, min_err_mtx, min_err_dist, min_err_rvecs,
            min_err_tvecs, min_err_idx)


"""

path = "/home/vortex/calib_ws_2/calibrationdata/calibration_data_right/"
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
            # Calibrobjpointsation
            ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints_right, imgpoints_right, gray.shape[::-1], None, None)

            # Undistortion
          #  h, w = img.shape[:2]
          #  new_camera_mtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
          #  dst = cv.undistort(img, mtx, dist, None, new_camera_mtx)

            # Crop the image
           # x, y, w, h = roi
           # dst = dst[y:y+h, x:x+h]
           # cv.imshow("calibrate_result", dst)
           # cv.waitKey(500)

            ret_list_right.append(ret)
            mtx_list_right.append(mtx)
            dist_list_right.append(dist)
            rvec_list_right.append(rvecs)
            tvec_list_right.append(tvecs)

           # print("ret: " + str(ret))
            #print("camera matrix: ", mtx)
            #print("dist: ", dist)

mean_error = 0
for i in range(len(objpoints_left)):
     imgpoints2, _ = cv.projectPoints(objpoints_left[i], rvecs[i], tvecs[i], mtx, dist)
     error = cv.norm(imgpoints_left[i], imgpoints2, cv.NORM_L2)/len(imgpoints2)
     mean_error += error
print( "total error: {}".format(mean_error/len(objpoints_left)) )
"""

# left_params = calib_cam(path_left)
# right_params = calib_cam(path_right)

# print("\n\n")
# print(left_params)
# print("\n\n")
# print(right_params)
# cv.CALIB_FIX_INTRINSIC

# cv.destroyAllWindows()

path_right = "/home/vortex/calib_ws_2/calibrationdata/calibration_data_right/"
directory_right = os.fsencode(path_right)

paths_right = sorted(os.listdir(directory_right))

path_left = "/home/vortex/calib_ws_2/calibrationdata/calibration_data_left/"
directory_left = os.fsencode(path_left)

paths_left = sorted(os.listdir(directory_left))
