import cv2
from matplotlib import lines 
import numpy as np
import matplotlib.pyplot as plt

# def change_contrast(I,alpha,beta):
#     new_image = np.zeros_like(I)
#     for x in range(I.shape[0]):
#         for y in range(I.shape[1]):
#             new_image[x,y] = np.clip(alpha*I[x,y] + beta, 0, 255)
#     # new_image = np.clip(I * alpha, 0 ,255)
#     # print(new_image.shape, I.shape)
#     return new_image

path1 = 'feature_detection/test_image/gate_day1_medium_yaw.png'
path2 = 'feature_detection/test_image/gate_day1_medium_yaw_second.png'
path3 = 'feature_detection/test_image/gate_day1_medium_yaw_third.png'
path4 = 'feature_detection/test_image/gate_day1_medium_yaw_fourth.png'
img = cv2.imread(path4)
img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# img_contrast_selfwritten = change_contrast(img_gray,2.3,0)
# height = img.shape[0]
# width = img.shape[1]
# rho = np.sqrt(height^2 + width^2)
# theta = np.pi

img_contrast = cv2.multiply(img_gray,2.3)
edges = cv2.Canny(img_contrast,50,200)

## HoughLinesP
linesP = cv2.HoughLinesP(edges,1,np.pi/180,20,50,10)
m = np.zeros(linesP.shape[0])
print(m.shape, type(linesP))

print(linesP[2][0])
lines_ver = np.zeros((1,4,1))
lines_hor = np.zeros((1,4,1))
if linesP is not None:
    for i in range(0, len(linesP)):
        l = linesP[i][0]
        ## Finding parallel lines 
        m[i] = (l[3]- l[1])/(l[2]-l[0])
        if m[i] == 0:  ## vertical lines
            cv2.line(img, (l[0], l[1]), (l[2], l[3]), (0,0,255), 3)
            lines_ver[i] = np.append(lines_ver, l)
        elif np.abs(m[i]) == np.Inf: ## horizontal lines
            cv2.line(img, (l[0], l[1]), (l[2], l[3]), (0,255,0), 3)
            lines_hor.append(linesP[i][0])
# print(m)

## doesn't work for 
print(max(lines_ver[:][1]))





## HoughLines
# lines = cv2.HoughLines(edges,2,np.pi/180,150,1,0)
# if lines is not None:
#     for i in range(0, len(lines)):
#         rho = lines[i][0][0]
#         theta = lines[i][0][1]
#         a = np.cos(theta)
#         b = np.sin(theta)
#         x0 = a * rho
#         y0 = b * rho
#         pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
#         pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
#         cv2.line(img, pt1, pt2, (0,0,255), 3, cv2.LINE_AA)

# img_blur = cv2.blur(img_gray, (1,1)) # welche Kernel Size ist für diese Anwendung passend? Gar keine am besten

# edges = cv2.Canny(img_blur,100,200)

# print(edges)
stack_edges = np.hstack((edges))
stacked = np.hstack((img_gray, img_contrast))
cv2.imshow('Kanten', img)
cv2.waitKey(0)
# cv2.imshow('Gate', stacked)
# cv2.waitKey(0)
# plt.xlim([0, img.shape[1]])
# plt.ylim([img.shape[0], 0])
# plt.title('Dominant lines')
