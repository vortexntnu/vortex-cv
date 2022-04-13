import cv2
from matplotlib import lines 
import numpy as np
import matplotlib.pyplot as plt
from copy import deepcopy

''' ToDo
+ tuning of the parameter of cv2.HoughLinesP makes it more robust, but less sensitive
+ adapt algorithm to calibrated images --> change function lines_coord 
+ component detection --> some kind of filter(?) has to be applied

'''

# feature detection based on hough transform
class HoughMajingo:

    @staticmethod
    def lines_coord(lines,set,e):
        '''
        args: 
            lines:  set of lines
            set:    look for horizontal or vertical lines
            e:      margin between lines that are recognized as one

        return:
            rect_list:  returns starting and end point of detected line that satisfy the margin
            x_position: returns the mean of the "position" of the line 
                        -> vertical line: x_position, horizontal line: y_position
        '''
        # set = 0: vertical lines
        # set = 1: horizontal lines
        if set == 0:
            ver = 1  # value for vertical lines
        elif set ==1:
            ver = 0  # value for horizontal lines

        # e = 8 ## should be default value later
        lines_sort = lines[lines[:,0,set].argsort()]  
        x_all_lines = lines_sort[:,0,set]
        x_position = np.zeros((20,1),dtype = int)
        rect_list = np.zeros((20,1,4),dtype = int)

        k= 0
        j= 0
        
        # list of lines is filtered and lines that are close together are combined -> change this if it is not working with calibrated images
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

    @staticmethod
    def cut_zeros(list):
        '''
        deletes "empty" entries of the list (instead of line zero vector)
        arg:
            list: list of lines with "empty" entries
        return:
            list: list of lines 
        '''
        k=0
        original_list = list
        for i in range(len(original_list)):
            if np.all(original_list[i,:,:] == np.zeros((1,4))):
                list = np.delete(list ,k ,axis = 0)
                k -=1
            k +=1

        return list

    @staticmethod
    def connect_lines2bb(lines,set, distance):
        '''
        Two neighbouring lines detected from a bounding box. Due to sorted list
        the calculation of the distance to neighbouring lines is sufficient.

        args: 
            lines:  set of lines
            set:    look for horizontal (set=1) or vertical lines (set=0)
        return: 
            bb_corner_pair: two lines that form a bounding box
                    format: [left line, right line]
                    format of line [x_low, y_low, x_high, y_high]
        '''

        # initialize the line vector
        line_1 = lines[:-1,:,:]
        line_2 = lines[1:,:,:]

        # calculating distance between lines 
        dis = np.zeros((line_1.shape[0],1))
        for j in range(len(line_1)):
            dis[j] = line_2[j,0,set] -line_1[j,0,set]
        # print(dis)
        bb_corner_pair = []
        placeholder = np.zeros((8), dtype=int)
        # j =0
        for i in range(len(dis)):
            if i < len(dis)-1 and dis[i] < dis[i+1]:
                if dis[i] < distance:
                    # first pair of lines belong together
                    placeholder[:4] = line_1[i,0,:]
                    placeholder[4:] = line_2[i,0,:]
                    bb_corner_pair.append(placeholder)
                    placeholder = np.zeros((8), dtype=int)
            elif len(dis) == 1 and dis < distance: 
                placeholder[:4] = line_1[i,0,:]
                placeholder[4:] = line_2[i,0,:]
                bb_corner_pair.append(placeholder)
        return bb_corner_pair

    def centroid(self, list_bb):
        '''
        Calculates the centroid of the bounding boxes 
        arg: 
            list_bb: list of bounding box entries
                     one bounding box entry consists of one row vector
                        left line [x_low, y_low, x_high, y_high], right line [x_low, y_low, x_high, y_high]
        return: 
            centroid_list: centroid of each bounding box entry
        '''
        centroid_list = []
        cent_array = np.zeros((2))
        # print('in centroid calculation', len(list_bb), np.shape(list_bb))
        for i in range(len(list_bb)):
            if np.shape(list_bb)[1] == 8:
                bb = list_bb[i]  
                # print('Ich bin bb',bb)
                centroid_x = (bb[0]+ bb[6] + bb[2] + bb[4])//4
                centroid_y = (bb[1]+ bb[7] + bb[3] + bb[5]) //4
                # cent_array[0] = centroid_x
                # cent_array[1] = centroid_y

                centroid_list.append([centroid_x, centroid_y])
                # print('centroid list',centroid_list)
        
        return centroid_list

    @staticmethod
    def main(orig_img, t1, t2):
        '''
        Applying hough transform on the image and postprocess the results to get bounding boxes from the detected object
        arg: 
            orig_img: image in gray scale
            t1: lower threshold for canny edge detector
            t2: upper threshold for canny edge detector
        return: 
            bb: list of bounding boxes
            center: list of centroids; list consists of arrays with x and y entries belonging to the bounding box of same index
            img_gray: original image with drawn bounding boxes
            edges: image of edges --> delete, just for testing purposes
            img_contrast: contrasted image --> delete, just for testing purposes, preprocessing is done somewhere else
        '''
        img_gray = deepcopy(orig_img)
        visual_lines =  False # True # 
        
        ## Preprocessing: Edge detection 
        edges = cv2.Canny(img_gray,t1,t2)

        ## HoughLinesP
        linesP = cv2.HoughLinesP(edges,1,np.pi/180,30,minLineLength=25,maxLineGap=10)

        ## Orientation based Filtering 
        # m = np.zeros(linesP.shape[0]) ## replace this with orientation of the drone
        k = 0
        j = 0
        lines_ver = linesP
        lines_hor = linesP 
        if linesP is not None:
            for line_idx in range(0, len(linesP)):
                line = linesP[line_idx][0]
                # m[line_idx] = (line[3]- line[1]) / (line[2]-line[0]) --> not needed anymore, new solution should be validated
                # print((line[3]- line[1]) / (0))
                # print(m[line_idx])
                if (line[3]- line[1]) != 0:  #m[line_idx] != 0: ## horizontal lines: processing
                    lines_hor = np.delete(lines_hor,k, axis= 0)
                    k -=1
                if (line[2]-line[0]) != 0: # (line[2]-line[0]) < 10^(-20) and (line[2]-line[0]) > -10^(-20):   #(line[2]-line[0]) != 0: #np.abs(m[line_idx]) != np.Inf: ## vertical lines: processing
                    lines_ver = np.delete(lines_ver,j, axis= 0)
                    j -=1
    
                k +=1
                j +=1
        
        if visual_lines:
            ## Visualization of detected hough lines
            if lines_hor is not None:
                for i in range(len(lines_hor)):
                    line_hor = lines_hor[i,0,:]
                    cv2.line(img_gray, (line_hor[0], line_hor[1]), (line_hor[2], line_hor[3]), (0,0,255), 2)
            
            if lines_ver is not None:
                for i in range(len(lines_ver)):
                    line_ver = lines_ver[i,0,:]
                    cv2.line(img_gray, (line_ver[0], line_ver[1]), (line_ver[2], line_ver[3]), (0,255,0), 2)
        

        ## processing vertical and horizontal lines
        rect_list_ver, pos_ver = HoughMajingo.lines_coord(lines_ver, 0, 5)
        rect_list_hor, pos_hor = HoughMajingo.lines_coord(lines_hor, 1, 5)

        ## cut zeros from lists
        rect_list_ver_new = HoughMajingo.cut_zeros(rect_list_ver)
        rect_list_hor_new = HoughMajingo.cut_zeros(rect_list_hor)


        ## correlating lines and getting corner points from bounding box --> both should be outputted
        distanz = 35
        if len(rect_list_ver_new) >1:
            bb_ver = HoughMajingo.connect_lines2bb(rect_list_ver_new, 0, distanz)
        else:
            bb_ver = None
        if len(rect_list_hor_new) >1:
            bb_hor = HoughMajingo.connect_lines2bb(rect_list_hor_new, 1, distanz)
        else:
            bb_hor = None

        bb = []
        bbound = False
        if bb_ver is not None:
            if np.shape(bb_ver) != 0: #
                bb = bb_ver
                bbound = True

        if bb_hor is not None:
            if np.shape(bb_ver) != 0:
                bb = bb + bb_hor
            else: 
                bb = bb_hor
            bbound = True
        
        Hough = HoughMajingo()
        if bbound == True and bb:
            center = Hough.centroid(bb)
        else: 
            center = None

        
        ## instead of gate/ feature detection component detection should be used if gate couldn't be detected
        ## for component detection the distance from drone to object has to be below a specific value
        component = False
        
        if len(bb) == 0 :
            # print('Component detection',len(bb))
            ### Check if distance to line is big --> if z of position of detected line > 8:
            ### Component is outputed 
            platzhalter = np.zeros(4)
            if len(rect_list_ver_new) != 0:
                for i in range(len(rect_list_ver_new)):
                    platzhalter[:4] = rect_list_ver_new[i,0,:]
                    bb.append(platzhalter)
                component = True
            if len(rect_list_hor_new) != 0:
                for i in range(len(rect_list_hor_new)):
                    platzhalter[:4] = rect_list_hor_new[i,0,:]
                    bb.append(platzhalter)
                component = True
        
        ####### Visualization of postprocessed hough lines
        # Visualization of vertical lines
        if visual_lines:
            for line_idx in range(len(pos_ver)):
                if pos_ver[line_idx] != 0:
                    line_ver = rect_list_ver[line_idx,0,:]
                    cv2.line(img_gray, (line_ver[0], line_ver[1]), (line_ver[2], line_ver[3]), (255,0,255), 4)
            
            # Visualization of horizontal lines
            for line_idx in range(len(pos_hor)):
                if pos_hor[line_idx] != 0:
                    line_hor = rect_list_hor[line_idx,0,:]
                    cv2.line(img_gray, (line_hor[0], line_hor[1]), (line_hor[2], line_hor[3]), (255,255,0), 4)

        # Visualization of bounding boxes
        # if component:
        # print('New Bounding boxes', bb, 'shape',np.shape(bb)) 
        if component == False:
            if np.shape(bb)[0] > 0:
                for line_idx in range(len(bb)):
                    line = bb[line_idx] 
                    if np.shape(line) != 0:
                        cv2.line(img_gray, (line[0], line[1]), (line[2], line[3]), (255,255,255), 2)
                        cv2.line(img_gray, (line[0], line[1]), (line[4], line[5]), (255,255,255), 2)
                        cv2.line(img_gray, (line[2], line[3]), (line[6], line[7]), (255,255,255), 2)
                        cv2.line(img_gray, (line[4], line[5]), (line[6], line[7]), (255,255,255), 2)
                
                if center is not None:
                    for cent_idx in range(len(center)): 
                        cv2.circle(img_gray, (int(center[cent_idx][0]),int(center[cent_idx][1])), radius=3, color=(0, 0, 255), thickness=-1)
        # print('center in the end',center)
        return bb, center, img_gray, edges