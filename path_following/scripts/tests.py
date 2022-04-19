
import feature_detection
import cv2
import numpy as np


def find_contour_line(contour, img_drawn):
    vx, vy, x0, y0 = cv2.fitLine(contour, cv2.DIST_L2, 0, 0.1, 0.1)

    colin_vec = np.ravel(np.array((vx, vy)))
    p0 = np.ravel(np.array((x0, y0)))
    
    p1 = p0 + 1000*colin_vec
    p2 = p0 - 1000*colin_vec

    cv2.line(img_drawn, p1.astype(int), p2.astype(int), color=(0, 255, 0), thickness=2)

    return [vx, vy, x0, y0], img_drawn

img = cv2.imread("./data/path_bendy_full.png")

feature_detector = feature_detection.FeatureDetection(np.shape(img))

hsv_params = [0,       #hsv_hue_min
              53,     #hsv_hue_max
              71,       #hsv_sat_min
              131,     #hsv_sat_max
              175,       #hsv_val_min
              248]     #hsv_val_max


ksize1 = 7
ksize2 = 7
sigma = 0.8
thresholding_blocksize = 11
thresholding_C = 2
#thresholdingsion and dilation params
erosion_dilation_ksize = 5
erosion_iterations = 1
dilation_iterations = 1
noise_rm_params = [ksize1, ksize2, sigma, thresholding_blocksize, thresholding_C, erosion_dilation_ksize, erosion_iterations, dilation_iterations]


_, hsv_mask, hsv_mask_validation_img = feature_detector.hsv_processor(img, 
                                    *hsv_params)

noise_filtered_img = feature_detector.noise_removal_processor(hsv_mask, *noise_rm_params)

path_contour = feature_detector.contour_processing(noise_filtered_img, contour_area_threshold=100, variance_filtering=True, coloured_img=img, return_image=False)

cv2.drawContours(img, path_contour, -1, (0,0,255), 5)

M = cv2.moments(path_contour)
cx_tilde = int(M['m10']/M['m00'])
cy_tilde = int(M['m01']/M['m00'])
#X = (self.Z_prior / self.focal_length) * cx_tilde
#Y = (self.Z_prior / self.focal_length) * cy_til
#dp_ref = [X, Y, self.Z_prior]

upper_inds = np.where((path_contour[:,:,1] < cy_tilde) == True)
upper_contour = path_contour[upper_inds[0]]

line_params, img_drawn = find_contour_line(upper_contour, img)

#path_contour[:,:,1]
cv2.imshow("Path", img_drawn)
cv2.waitKey(0)