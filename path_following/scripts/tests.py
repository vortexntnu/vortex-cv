
import feature_detection
import cv2
import numpy as np

img = cv2.imread("./data/path_bendy_full.png")

feature_detector = feature_detection.FeatureDetection(np.shape(img))

hsv_hue_max = 179
hsv_hue_min = 0
hsv_sat_max = 231
hsv_sat_min = 115
hsv_val_max = 172
hsv_val_min = 59

hsv_hue_max = 15
hsv_hue_min = 4
hsv_sat_max = 244
hsv_sat_min = 137
hsv_val_max = 230
hsv_val_min = 116

hsv_params = [hsv_hue_max, hsv_hue_min, hsv_sat_max, hsv_sat_min, hsv_val_max, hsv_val_min]

_, mask, hsv_mask_validation_img = feature_detector.hsv_processor(img, 
                                    hsv_params[0], hsv_params[1], hsv_params[2],
                                    hsv_params[3], hsv_params[4], hsv_params[5])


cv2.imshow("Path", hsv_mask_validation_img)
cv2.waitKey(0)