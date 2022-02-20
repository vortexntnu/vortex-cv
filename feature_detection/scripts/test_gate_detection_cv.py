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

print_picture = True

path1 = 'feature_detection/test_image/gate_day1_medium_yaw.png'
path2 = 'feature_detection/test_image/gate_day1_medium_yaw_second.png'
path3 = 'feature_detection/test_image/gate_day1_medium_yaw_third.png'
path4 = 'feature_detection/test_image/gate_day1_medium_yaw_fourth.png'
img = cv2.imread(path3)
img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# img_contrast_selfwritten = change_contrast(img_gray,2.3,0)
# height = img.shape[0]
# width = img.shape[1]
# rho = np.sqrt(height^2 + width^2)
# theta = np.pi

img_contrast = cv2.multiply(img_gray,2.3) ## contrast factor should be adaptable to distance and brightness
edges = cv2.Canny(img_contrast,50,200)

## HoughLinesP
linesP = cv2.HoughLinesP(edges,1,np.pi/180,20,50,10)
m = np.zeros(linesP.shape[0])
# print(linesP.shape, type(linesP))

# print(linesP) #[2][0]

## mix- up in the choice of the horizontal and vertical lines
k = 0
j = 0
lines_ver = linesP # np.zeros((1,4,1))
lines_hor = linesP # np.zeros((1,4,1))
if linesP is not None:
    for i in range(0, len(linesP)):
        l = linesP[i][0]
        m[i] = (l[3]- l[1])/(l[2]-l[0])
        if m[i] == 0:  ## vertical lines: visualization
            cv2.line(img, (l[0], l[1]), (l[2], l[3]), (0,0,255), 3)
        if m[i] != 0: ## vertical lines: processing
            lines_hor = np.delete(lines_hor,k, axis= 0)
            k -=1
        if np.abs(m[i]) == np.Inf: ## horizontal lines: visualization
            cv2.line(img, (l[0], l[1]), (l[2], l[3]), (0,255,0), 3)
        if np.abs(m[i]) != np.Inf: ## horizontal lines: processing
            lines_ver = np.delete(lines_ver,j, axis= 0)
            j -=1
        k +=1
        j +=1
print(lines_ver)

## processing vertical lines
# 1. ungefähre position der Pfeiler herausbekommen: 4 verschiedene x-positionen
# 2. obersten punkt der vertikalen Linien ermitteln 
# 3. obersten Punkt zum Filtern der horizontalen Linien bestimmen --> Zuverlässigkeit und Genauigkeit definieren
# print(lines_ver, max(lines_ver[:][0][0]))

## processing vertical lines
# print(lines_ver[:,0,0])
e = 10
x_vertical = np.sort(lines_ver[:,0,0])
vote = np.zeros_like(x_vertical)
# Positionen der Pfeiler ermitteln
for i in range(x_vertical.shape[0]):
    for j in range(x_vertical.shape[0]):
        if i != j:
            # print(x_vertical[i],x_vertical[j] -e/2, x_vertical[j] +e/2)
            if x_vertical[i] >= x_vertical[j] -e/2 and x_vertical[i] <= x_vertical[j] +e/2:
                # print(x_vertical[i], x_vertical[j])
                vote[i] +=1

#         x_ll = min(lines_ver[:,0,0])
#         x_rr = max(lines_ver[:,0,0])
# print(x_vertical, x_ll, x_rr,x_vertical.shape[0])
print(vote, x_vertical)
# identify inidizes which are equal to zero --> should be changed to votes that are equally high instead of zero
x_position = np.zeros((9,1))
gr = vote[vote == 0].shape
index = np.zeros(gr)

k=0
for i in range(vote.shape[0]):
    if vote[i] == 0: 
        index[k] = int(i)
        k +=1

i = 0
for j in range(index.shape[0]-1):
    if index[j] != index[j+1] +1:
    
        start = int(index[j])
        ende = int(index[j+1])
        print(start, ende, x_vertical[start:ende])
        if len(x_vertical[start:ende]) > 1:
            x_position[i] = np.mean(x_vertical[start:ende], dtype = int)
            i +=1
                # break

print(x_position)
## Was passiert, wenn nur ein Teil des Gates sichtbar ist? Nur linke oder nur rechte? Kann man darüber eine Aussage treffen?

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

for i in range(len(x_position)):
    if x_position[i] != 0:
        cv2.line(img, (x_position[i], 0), (x_position[i], 1000), (255,255,0), 3)

# print(edges)
if print_picture:
    stack_edges = np.hstack((edges))
    stacked = np.hstack((img_gray, img_contrast))
    cv2.imshow('Kanten', img)
    cv2.waitKey(0)


# cv2.imshow('Gate', stacked)
# cv2.waitKey(0)
# plt.xlim([0, img.shape[1]])
# plt.ylim([img.shape[0], 0])
# plt.title('Dominant lines')
