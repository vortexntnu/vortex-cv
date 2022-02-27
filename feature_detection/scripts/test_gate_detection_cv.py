from cmath import rect
from pickletools import uint8
import cv2
from matplotlib import lines 
import numpy as np
import matplotlib.pyplot as plt

def lines_coord(lines,set,e):
    '''
    args: 
        lines: set of lines
        set: look for horizontal or vertical lines
    '''
    # set = 0: vertical lines
    # set = 1: horizontal lines
    if set == 0:
        ver = 1
    elif set ==1:
        ver = 0

    # e = 8 ## should be default value later
    lines_sort = lines[lines[:,0,set].argsort()]  
    x_all_lines = lines_sort[:,0,set]
    x_position = np.zeros((20,1))
    rect_list = np.zeros((20,1,4),dtype = int)

    k= 0
    j= 0
    for i in range(x_all_lines.shape[0]-1):
        if x_all_lines[i+1] - x_all_lines[i] >= e or i == x_all_lines.shape[0]-2: 
            if len(x_all_lines[k:i]) >1:
                x_pos_mean = int(x_all_lines[k:i+1].mean())
                y_pos_ver =  lines_sort[k:i+1, 0, ver]
                y_pos_min = min(y_pos_ver)
                y_pos_ver =  lines_sort[k:i+1, 0,ver +2]
                y_pos_max = max(y_pos_ver)
                k = i +1
                x_position[j] = x_pos_mean
                rect_list[j,0,set] = int(x_pos_mean)
                rect_list[j,0,ver] = int(y_pos_min)
                rect_list[j,0,set+2] = int(x_pos_mean)
                rect_list[j,0,ver+2] = int(y_pos_max)

                
                j +=1
            else:
                k += 1
    return rect_list, x_position
def cut_zeros(list):
    beispiel = np.zeros((1,4))
    k=0
    original_list = list
    for i in range(len(original_list)):
        if np.all(original_list[i,:,:] == beispiel):
            list = np.delete(list ,k ,axis = 0)
            k -=1
        k +=1
    return list

def connect_lines2bb(lines,set):
    '''
    args: 
        lines: set of lines
        set: look for horizontal or vertical lines
    '''
    # set = 0: vertical lines (the coordinate that is the same)
    # set = 1: horizontal lines

    ## aufgrund des Sortierens der Linien im Vorhinein ist die Berechnung der Distanz zu benachbarten Linien ausreichend

    # calculating distance between lines 
    # initialize the line vector
    line_1 = lines[:-1,:,:]
    line_2 = lines[1:,:,:]


    dis = np.zeros((line_1.shape[0],1))
    for i in range(len(line_1)): #zip(range(len(line_1)),range(len(line_2)))
        dis[i] = line_2[i,0,set] -line_1[i,0,set]
    # print(dis)
    print(len(line_1))
    bb_corner_pair = []
    platzhalter = np.zeros((1,1,8))
    j =0
    for i in range(len(dis)):
        print(len(dis)-1,i)
        if i < len(dis)-1 and dis[i] < dis[i+1]:
            # first pair of lines belong together
            print(line_1[i,0,:],line_2[i,0,:])
            platzhalter[0,0,:4] = line_1[i,0,:]
            platzhalter[0,0,4:] = line_2[i,0,:]
            bb_corner_pair.append(platzhalter)
            platzhalter = np.zeros((1,1,8))
            print(bb_corner_pair)
        elif i == len(dis)-1 and dis[i] < dis[i-1]:
            print(line_1[i,0,:])
            # last pair of lines belong together
            platzhalter[0,0,:4] = line_1[i,0,:]
            platzhalter[0,0,4:] = line_2[i,0,:]
            bb_corner_pair.append(platzhalter)
            print(bb_corner_pair)
        
    return bb_corner_pair


print_picture = False

path1 = 'feature_detection/test_image/gate_day1_medium_yaw.png'
path2 = 'feature_detection/test_image/gate_day1_medium_yaw_second.png'
path3 = 'feature_detection/test_image/gate_day1_medium_yaw_third.png'
path4 = 'feature_detection/test_image/gate_day1_medium_yaw_fourth.png'
path= [path1, path2, path3, path4]
for i in range(1): # 4 for all pictures
    
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

    ## Orientation based Filtering 
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

    ## processing vertical and horizontal lines
    rect_list_ver, x_position = lines_coord(lines_ver, 0,8)
    rect_list_hor, pos_hor = lines_coord(lines_hor, 1, 5)

    ## cut zeros from lists!!!
    rect_list_ver_new = cut_zeros(rect_list_ver)
    rect_list_hor_new = cut_zeros(rect_list_hor)

    ## correlating lines and getting corner points from bounding box
    bb_ver = connect_lines2bb(rect_list_ver_new, 0)
    print(bb_ver)

    ## Visualization
    for i in range(len(x_position)):
        if x_position[i] != 0:
            line_ver = rect_list_ver[i,0,:]
            cv2.line(img, (line_ver[0], line_ver[1]), (line_ver[2], line_ver[3]), (255,0,255), 3)
            # cv2.line(img, (x_position[i], 0), (x_position[i], 1000), (255,255,0), 3)
    
    for i in range(len(pos_hor)):
        if pos_hor[i] != 0:
            line_hor = rect_list_hor[i,0,:]
            # print(line_hor)
            cv2.line(img, (line_hor[0], line_hor[1]), (line_hor[2], line_hor[3]), (255,255,0), 4)
            # cv2.line(img, (pos_hor[i], 0), (pos_hor[i], 1000), (255,255,0), 3)

    if print_picture:
        # stack_edges = np.hstack((edges,edges2))
        stacked = np.hstack((img_gray, img_contrast))
        cv2.imshow('Kanten', img)
        cv2.waitKey(0)



