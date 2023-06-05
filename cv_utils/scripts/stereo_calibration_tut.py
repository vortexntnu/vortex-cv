import numpy as np
import cv2 as cv
import glob


def update_config(path, newCameraMatrixL, distL, newCameraMatrixR, distR,
                  resolution):

    print("Using resolution: ", resolution)

    resolutions = {
        "(2560,1440)": "LEFT_CAM_2K",
        "(1920,1080)": "[LEFT_CAM_FHD]",
        "(1280, 720)": "[LEFT_CAM_HD]",
        "(672, 376)": "[LEFT_CAM_VGA]"
    }

    try:
        if (resolutions[resolution] == None):
            print("No such resolution")
            raise Exception("No such resolution")

        print(resolutions[resolution])
        with open(path, "r") as c:
            config = c.readlines()

        j = 0
        for i in range(len(config)):
            if config[i].strip() == resolutions[resolution]:
                j = i
                found = True

        resolution_string = resolutions[resolution][10:]

        if found:
            j += 1
            config[j] = f"fx={newCameraMatrixL[0][0]}\n"
            j += 1
            config[j] = f"fy={newCameraMatrixL[1][1]}\n"
            j += 1
            config[j] = f"cx={newCameraMatrixL[0][2]}\n"
            j += 1
            config[j] = f"cy={newCameraMatrixL[1][2]}\n"
            j += 1
            config[j] = f"k1={distL[0][0]}\n"
            j += 1
            config[j] = f"k2={distL[0][1]}\n"
            j += 1
            config[j] = f"k3={distL[0][4]}\n"
            j += 1
            config[j] = f"p1={distL[0][2]}\n"
            j += 1
            config[j] = f"p2={distL[0][3]}\n"
            j += 1
            config[j] = "\n"
            j += 1
            config[j] = f"[RIGHT_CAM_{resolution_string}\n"
            j += 1
            config[j] = f"fx={newCameraMatrixR[0][0]}\n"
            j += 1
            config[j] = f"fy={newCameraMatrixR[1][1]}\n"
            j += 1
            config[j] = f"cx={newCameraMatrixR[0][2]}\n"
            j += 1
            config[j] = f"cy={newCameraMatrixR[1][2]}\n"
            j += 1
            config[j] = f"k1={distR[0][0]}\n"
            j += 1
            config[j] = f"k2={distR[0][1]}\n"
            j += 1
            config[j] = f"k3={distR[0][4]}\n"
            j += 1
            config[j] = f"p1={distR[0][2]}\n"
            j += 1
            config[j] = f"p2={distR[0][3]}\n"
            j += 1

            config[j] = "\n"

            with open(path, "w") as f:
                for line in config:
                    f.write(line)
                f.close()
            for line in config:
                print(line)

    except:
        print(
            "An error occurred. The function was given an invalid resolution")
        print("Valid resolutions are: \n")
        for key in resolutions.keys():
            print(key)


# Calibration file path: On Linux: /usr/local/zed/settings/
################ FIND CHESSBOARD CORNERS - OBJECT POINTS AND IMAGE POINTS #############################

resolutions = {
    "FHD": (1920, 1080),
    "2K": (2560, 1440),
    "HD": (1280, 720),
    "VGA": (672, 376)
}
chessboardSize = (5, 7)

# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((chessboardSize[0] * chessboardSize[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboardSize[0],
                       0:chessboardSize[1]].T.reshape(-1, 2)
objp *= 108
# print(objp)

# Arrays to store object points and image points from all the images.
objpoints = []  # 3d point in real world space
imgpointsL = []  # 2d points in image plane.
imgpointsR = []  # 2d points in image plane.

imagesLeft = glob.glob(
    '/home/vortex/cv_ws/src/Vortex_CV/camera_calibration/calibrationdata/ros_underwater_left/*.jpg'
)
imagesRight = glob.glob(
    '/home/vortex/cv_ws/src/Vortex_CV/camera_calibration/calibrationdata/ros_underwater_right/*.jpg'
)
config_path = "/home/vortex/cv_ws/src/Vortex_CV/camera_calibration/scripts/SN38762967FACTORY_REAL_THIS_TIME_FU_BENJAMIN.conf"

for i, images in enumerate(sorted(zip(imagesLeft, imagesRight))):
    if i % 10 == 0:
        imgLeft, imgRight = images
        imgL = cv.imread(imgLeft)
        frameSize = imgL.shape[:2]
        print(frameSize)
        # print(imgL.shape)
        imgR = cv.imread(imgRight)
        grayL = cv.cvtColor(imgL, cv.COLOR_BGR2GRAY)
        grayR = cv.cvtColor(imgR, cv.COLOR_BGR2GRAY)

        # Find the chess board corners
        retL, cornersL = cv.findChessboardCorners(grayL, chessboardSize, None)
        retR, cornersR = cv.findChessboardCorners(grayR, chessboardSize, None)

        # print(retL)
        # print(retR)

        # If found, add object points, image points (after refining them)
        if retL and retR == True:
            print(imgLeft)

            objpoints.append(objp)

            cornersL = cv.cornerSubPix(grayL, cornersL, (11, 11), (-1, -1),
                                       criteria)
            imgpointsL.append(cornersL)

            cornersR = cv.cornerSubPix(grayR, cornersR, (11, 11), (-1, -1),
                                       criteria)
            imgpointsR.append(cornersR)

            # Draw and display the corners
            # cv.drawChessboardCorners(imgL, chessboardSize, cornersL, retL)
            # cv.imshow('img left', imgL)
            # cv.drawChessboardCorners(imgR, chessboardSize, cornersR, retR)
            # cv.imshow('img right', imgR)
            # cv.waitKey(1000)

cv.destroyAllWindows()

############## CALIBRATION #######################################################

retL, cameraMatrixL, distL, rvecsL, tvecsL = cv.calibrateCamera(
    objpoints, imgpointsL, frameSize, None, None)

# print("Calib Left:", retL, cameraMatrixL, distL, rvecsL, tvecsL)
heightL, widthL, channelsL = imgL.shape
newCameraMatrixL, roi_L = cv.getOptimalNewCameraMatrix(cameraMatrixL, distL,
                                                       (widthL, heightL), 1,
                                                       (widthL, heightL))

retR, cameraMatrixR, distR, rvecsR, tvecsR = cv.calibrateCamera(
    objpoints, imgpointsR, frameSize, None, None)
# print("Calib Right", retR, cameraMatrixR, distR, rvecsR, tvecsR)
heightR, widthR, channelsR = imgR.shape
newCameraMatrixR, roi_R = cv.getOptimalNewCameraMatrix(cameraMatrixR, distR,
                                                       (widthR, heightR), 1,
                                                       (widthR, heightR))

########## Stereo Vision Calibration #############################################

flags = 0
flags |= cv.CALIB_FIX_INTRINSIC
# Here we fix the intrinsic camara matrixes so that only Rot, Trns, Emat and Fmat are calculated.
# Hence intrinsic parameters are the same

criteria_stereo = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# This step is performed to transformation between the two cameras and calculate Essential and Fundamenatl matrix
retStereo, newCameraMatrixL, distL, newCameraMatrixR, distR, rot, trans, essentialMatrix, fundamentalMatrix = cv.stereoCalibrate(
    objpoints, imgpointsL, imgpointsR, newCameraMatrixL, distL,
    newCameraMatrixR, distR, grayL.shape[::-1], criteria_stereo, flags)

update_config(config_path, newCameraMatrixL, distL, newCameraMatrixR, distR,
              str((frameSize[1], frameSize[0])))

rod = cv.Rodrigues(rot, jacobian=False)
print("rod", rod)

rect_image_left = cv.undistort(imgL, newCameraMatrixL, distL)
rect_image_right = cv.undistort(imgR, newCameraMatrixR, distR)

# Crop:
# x, y, w, h = roi_L
# rect_image_left = rect_image_left[y:y+h, x:x+w]

########## Stereo Rectification #################################################

rectifyScale = 1
rectL, rectR, projMatrixL, projMatrixR, Q, roi_L, roi_R = cv.stereoRectify(
    newCameraMatrixL, distL, newCameraMatrixR, distR, grayL.shape[::-1], rot,
    trans, rectifyScale, (0, 0))

stereoMapL = cv.initUndistortRectifyMap(newCameraMatrixL, distL, rectL,
                                        projMatrixL, grayL.shape[::-1],
                                        cv.CV_16SC2)
stereoMapR = cv.initUndistortRectifyMap(newCameraMatrixR, distR, rectR,
                                        projMatrixR, grayR.shape[::-1],
                                        cv.CV_16SC2)

print("Saving parameters!")
cv_file = cv.FileStorage(
    '/home/vortex/vortex_ws/src/Vortex_CV/camera_calibration/calibrationdata/calib_data_with_xml/zed_image_list.xml',
    cv.FILE_STORAGE_WRITE)

cv_file.write('stereoMapL_x', stereoMapL[0])
cv_file.write('stereoMapL_y', stereoMapL[1])
cv_file.write('stereoMapR_x', stereoMapR[0])
cv_file.write('stereoMapR_y', stereoMapR[1])

cv_file.release()
