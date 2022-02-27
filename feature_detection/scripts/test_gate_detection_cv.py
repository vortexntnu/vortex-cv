from cmath import rect
from pickletools import uint8
import cv2
from matplotlib import lines 
import numpy as np
import matplotlib.pyplot as plt


print_picture = True

path1 = 'feature_detection/test_image/gate_day1_medium_yaw.png'
path2 = 'feature_detection/test_image/gate_day1_medium_yaw_second.png'
path3 = 'feature_detection/test_image/gate_day1_medium_yaw_third.png'
path4 = 'feature_detection/test_image/gate_day1_medium_yaw_fourth.png'
path= [path1, path2, path3, path4]
for i in range(4):
    
    img = cv2.imread(path[i])
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    ## Preprocessing
    # Noise filtering
    kernel = np.ones((4,4), np.uint8)  
    img_dilation = cv2.dilate(img_gray, kernel, iterations=1)  
    img_erosion = cv2.erode(img_dilation, kernel, iterations=1)  
    img_dilation2 = cv2.dilate(img_erosion, kernel, iterations=1) 

    img_contrast = cv2.multiply(img_erosion,2.3) ## contrast factor should be adaptable to distance and brightness
    edges = cv2.Canny(img_contrast,50,200)

    # img_contrast2 = cv2.multiply(img_dilation2,2.3) ## contrast factor should be adaptable to distance and brightness
    # edges2 = cv2.Canny(img_contrast2,50,200)


    ## HoughLinesP
    linesP = cv2.HoughLinesP(edges,1,np.pi/180,20,50,10)
    m = np.zeros(linesP.shape[0]) ## replace this with orientation of the drone

    
    k = 0
    j = 0
    lines_ver = linesP
    lines_hor = linesP 
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
    
    ## processing vertical lines
    e = 8 
    lines_ver_sort = lines_ver[lines_ver[:,0,0].argsort()]  
    x_vertical = lines_ver_sort[:,0,0]
    x_position = np.zeros((9,1))
    rect_list = np.zeros((9,1,4),dtype = int)


    k= 0
    j= 0
    for i in range(x_vertical.shape[0]-1):
        if x_vertical[i+1] - x_vertical[i] >= e or i == x_vertical.shape[0]-2: 
            if len(x_vertical[k:i]) >1:
                x_pos_mean = int(x_vertical[k:i+1].mean())
                y_pos_ver =  lines_ver_sort[k:i+1, 0,1]
                y_pos_min = min(y_pos_ver)
                y_pos_max = max(y_pos_ver)
                k = i +1
                x_position[j] = x_pos_mean
                rect_list[j,0,0] = int(x_pos_mean)
                rect_list[j,0,1] = int(y_pos_min)
                rect_list[j,0,2] = int(x_pos_mean)
                rect_list[j,0,3] = int(y_pos_max)

                
                j +=1
            else:
                k += 1
    


    for i in range(len(x_position)):
        if x_position[i] != 0:
            line = rect_list[i,0,:]
            cv2.line(img, (line[0], line[1]), (line[2], line[3]), (255,255,0), 3)
            # cv2.line(img, (x_position[i], 0), (x_position[i], 1000), (255,255,0), 3)

    if print_picture:
        # stack_edges = np.hstack((edges,edges2))
        stacked = np.hstack((img_gray, img_contrast))
        cv2.imshow('Kanten', img)
        cv2.waitKey(0)



## processing vertical lines
    # 1. ungefähre position der Pfeiler herausbekommen: 4 verschiedene x-positionen
    # 2. obersten punkt der vertikalen Linien ermitteln 
    # 3. obersten Punkt zum Filtern der horizontalen Linien bestimmen --> Zuverlässigkeit und Genauigkeit definieren
    # print(lines_ver, max(lines_ver[:][0][0]))
