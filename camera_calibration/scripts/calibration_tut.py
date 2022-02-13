
import sys
import numpy as np
import time
import cv2

# Camera parameters to undistort and rectify images
cv_file = cv2.FileStorage()
cv_file.open('/home/vortex/vortex_ws/src/Vortex_CV/camera_calibration/calibrationdata/calib_data_with_xml/zed_image_list.xml', cv2.FileStorage_READ)

stereoMapL_x = cv_file.getNode('stereoMapL_x').mat()
stereoMapL_y = cv_file.getNode('stereoMapL_y').mat()
stereoMapR_x = cv_file.getNode('stereoMapR_x').mat()
stereoMapR_y = cv_file.getNode('stereoMapR_y').mat()

# pathL = "/home/vortex/vortex_ws/src/Vortex_CV/camera_calibration/calibrationdata/zed2i_left/zed_image_L14.png"
# pathR = "/home/vortex/vortex_ws/src/Vortex_CV/camera_calibration/calibrationdata/zed2i_right/zed_image_R14.png"
# 
# imgR = cv2.imread(pathR)
# imgL = cv2.imread(pathL)
# 
# cv2.imshow("imgR", imgR)
# cv2.waitKey(5000)
# cv2.imshow("imgL", imgL)
# cv2.waitKey(5000)

def undistortRectify(frameR, frameL):

    # Undistort and rectify images
    undistortedL= cv2.remap(frameL, stereoMapL_x, stereoMapL_y, cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)
    undistortedR= cv2.remap(frameR, stereoMapR_x, stereoMapR_y, cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)


    return undistortedR, undistortedL

# 
# undistortedR, undistortedL = undistortRectify(imgR, imgL)
# 

# cv2.imshow("undistortedR", undistortedR)
# cv2.waitKey(5000)

# cv2.imshow("unistortedL", undistortedL)
# cv2.waitKey(5000)