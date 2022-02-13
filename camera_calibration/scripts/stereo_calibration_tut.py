import numpy as np
import cv2 as cv
import glob



################ FIND CHESSBOARD CORNERS - OBJECT POINTS AND IMAGE POINTS #############################

resolutions = {"FHD":(1920,1080), "2K": (2560,1440), "HD": (1280, 720), "VGA": (672, 376) }

chessboardSize = (5,7)
frameSize = resolutions["FHD"]



# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)


# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((chessboardSize[0] * chessboardSize[1], 3), np.float32)
objp[:,:2] = np.mgrid[0:chessboardSize[0],0:chessboardSize[1]].T.reshape(-1,2)
objp *= 108
# print(objp)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpointsL = [] # 2d points in image plane.
imgpointsR = [] # 2d points in image plane.


imagesLeft = glob.glob('/home/vortex/vortex_ws/src/Vortex_CV/camera_calibration/calibrationdata/zed2i_left/*.png')
imagesRight = glob.glob('/home/vortex/vortex_ws/src/Vortex_CV/camera_calibration/calibrationdata/zed2i_right/*.png')

for imgLeft, imgRight in sorted(zip(imagesLeft, imagesRight)):

    imgL = cv.imread(imgLeft)
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

        cornersL = cv.cornerSubPix(grayL, cornersL, (11,11), (-1,-1), criteria)
        imgpointsL.append(cornersL)

        cornersR = cv.cornerSubPix(grayR, cornersR, (11,11), (-1,-1), criteria)
        imgpointsR.append(cornersR)

        # Draw and display the corners
        # cv.drawChessboardCorners(imgL, chessboardSize, cornersL, retL)
        # cv.imshow('img left', imgL)
        # cv.drawChessboardCorners(imgR, chessboardSize, cornersR, retR)
        # cv.imshow('img right', imgR)
        # cv.waitKey(1000)


cv.destroyAllWindows()




############## CALIBRATION #######################################################

retL, cameraMatrixL, distL, rvecsL, tvecsL = cv.calibrateCamera(objpoints, imgpointsL, frameSize, None, None)

# print("Calib Left:", retL, cameraMatrixL, distL, rvecsL, tvecsL)
heightL, widthL, channelsL = imgL.shape
newCameraMatrixL, roi_L = cv.getOptimalNewCameraMatrix(cameraMatrixL, distL, (widthL, heightL), 1, (widthL, heightL))

retR, cameraMatrixR, distR, rvecsR, tvecsR = cv.calibrateCamera(objpoints, imgpointsR, frameSize, None, None)
# print("Calib Right", retR, cameraMatrixR, distR, rvecsR, tvecsR)
heightR, widthR, channelsR = imgR.shape
newCameraMatrixR, roi_R = cv.getOptimalNewCameraMatrix(cameraMatrixR, distR, (widthR, heightR), 1, (widthR, heightR))



########## Stereo Vision Calibration #############################################

flags = 0
flags |= cv.CALIB_FIX_INTRINSIC
# Here we fix the intrinsic camara matrixes so that only Rot, Trns, Emat and Fmat are calculated.
# Hence intrinsic parameters are the same 

criteria_stereo= (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# This step is performed to transformation between the two cameras and calculate Essential and Fundamenatl matrix
retStereo, newCameraMatrixL, distL, newCameraMatrixR, distR, rot, trans, essentialMatrix, fundamentalMatrix = cv.stereoCalibrate(objpoints, imgpointsL, imgpointsR, newCameraMatrixL, distL, newCameraMatrixR, distR, grayL.shape[::-1], criteria_stereo, flags)

print("Stereo calib: ", retStereo, "\n", newCameraMatrixL, "\n", distL, "\n", newCameraMatrixR, "\n", distR, "\n ROT:", rot, "\n", trans, "\n", essentialMatrix, "\n", fundamentalMatrix)
#print(newCameraMatrixL)
#print(newCameraMatrixR)

with open("/home/vortex/vortex_ws/src/Vortex_CV/camera_calibration/scripts/calibration_params.txt", "w") as f:
    f.write("[LEFT_CAM_FHD]\n")
    f.write(f"fx={newCameraMatrixL[0][0]}\n")
    f.write(f"fy={newCameraMatrixL[1][1]}\n")
    f.write(f"cx={newCameraMatrixL[0][2]}\n")
    f.write(f"cy={newCameraMatrixL[1][2]}\n")
    f.write(f"k1={distL[0][0]}\n")
    f.write(f"k2={distL[0][1]}\n")
    f.write(f"k3={distL[0][4]}\n")
    f.write(f"p1={distL[0][2]}\n")
    f.write(f"p2={distL[0][3]}\n")
    f.write("\n")
    f.write("[RIGHT_CAM_FHD]\n")
    f.write(f"fx={newCameraMatrixR[0][0]}\n")
    f.write(f"fy={newCameraMatrixR[1][1]}\n")
    f.write(f"cx={newCameraMatrixR[0][2]}\n")
    f.write(f"cy={newCameraMatrixR[1][2]}\n")
    f.write(f"k1={distR[0][0]}\n")
    f.write(f"k2={distR[0][1]}\n")
    f.write(f"k3={distR[0][4]}\n")
    f.write(f"p1={distR[0][2]}\n")
    f.write(f"p2={distR[0][3]}\n")
    f.close()

R = (rot - rot.transpose())/2
print(R)

rz, ry, rx = -R[0][1], R[0][2], -R[1][2]

r = np.array([rx, ry, rz])
#print(np.linalg.norm(r))

R = R/np.sin(np.linalg.norm(r))
print("R: ", R)

# [LEFT_CAM_FHD]
# fx=1048.79517
# fy=1033.32812
# cx=943.372823
# cy=538.890956
# k1=-0.07983744
# k2= 0.02351426
# k3=-0.00783292
# p1=0.00024204
# p2=-0.00016786

# [RIGHT_CAM_FHD]
# fx=1.40242404e+02
# fy=1.39414274e+02
# cx=9.17294479e+01
# cy=5.62024701e+01
# k1=-1.26606067e-02
# k2=-6.89068565e-03
# k3=9.77296799e-02
# p1=-5.73110011e-05
# p2=-3.55594651e-04

########## Stereo Rectification #################################################

rectifyScale= 1
rectL, rectR, projMatrixL, projMatrixR, Q, roi_L, roi_R= cv.stereoRectify(newCameraMatrixL, distL, newCameraMatrixR, distR, grayL.shape[::-1], rot, trans, rectifyScale,(0,0))

stereoMapL = cv.initUndistortRectifyMap(newCameraMatrixL, distL, rectL, projMatrixL, grayL.shape[::-1], cv.CV_16SC2)
stereoMapR = cv.initUndistortRectifyMap(newCameraMatrixR, distR, rectR, projMatrixR, grayR.shape[::-1], cv.CV_16SC2)

print("Saving parameters!")
cv_file = cv.FileStorage('/home/vortex/vortex_ws/src/Vortex_CV/camera_calibration/calibrationdata/calib_data_with_xml/zed_image_list.xml', cv.FILE_STORAGE_WRITE)

cv_file.write('stereoMapL_x',stereoMapL[0])
cv_file.write('stereoMapL_y',stereoMapL[1])
cv_file.write('stereoMapR_x',stereoMapR[0])
cv_file.write('stereoMapR_y',stereoMapR[1])

cv_file.release()