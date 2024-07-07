#!/usr/bin/env python
# encoding: utf-8
import time
import rospy
import cv2 as cv
import numpy as np
from sensor_stop import *


class color_follow:
    def __init__(self):
        
        self.binary = None
        self.Center_x = -1
        self.Center_y = -1
        self.nigger = 0

    def adaptive_light(self,mysrc):
        src = mysrc
        M = 256 
        N = 256
        if src.ndim != 2 or M <= 0 or N <= 0:
            return None
        average = np.mean(src)
        rows_new = int(np.ceil(src.shape[0] / M))
        cols_new = int(np.ceil(src.shape[1] / N))
        #长宽可改
        D = np.zeros((rows_new, cols_new), dtype=np.float32)
        for i in range(rows_new):
            for j in range(cols_new):
                row_min = i * M
                row_max = min((i + 1) * M, src.shape[0])
                col_min = j * N
                col_max = min((j + 1) * N, src.shape[1])

                imageROI = src[row_min:row_max, col_min:col_max]
                temaver = np.mean(imageROI)
                D[i, j] = temaver
        E = D - average
        R = cv.resize(E, (src.shape[1], src.shape[0]), interpolation=cv.INTER_CUBIC)
        dst = src.astype(np.float32) - R
        result = dst.astype(np.uint8)
        return result

    def remove(self,binary,cnt_target,cnt_corss):
        find_contours = cv.findContours(binary, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        m_idx = -1
        contour = []
        if len(find_contours) == 3: contours = find_contours[1]
        else:
            contours = find_contours[0]
        if len(contours) != 0:
            area_m = []
            contours_info = []
            for j in range(len(contours)):
                area = cv.contourArea(contours[j])
                if area > 5000:
                    area_m.append(cv.contourArea(contours[j]))
                    cnt_y = contours[j][:,0,1]
                    y_max = max(cnt_y)
                    contour = contours[j]
                    (x, y), radius = cv.minEnclosingCircle(contour)
                    if cnt_target == 1 and cnt_corss == 1 or cnt_target == 3 and cnt_corss == 0:
                        contours_info.append({'distance': (abs(x-400)),'contour':contour,'y_max':y_max})
                    else:
                        contours_info.append({'distance': (abs(x-320)),'contour':contour,'y_max':y_max})
                else:
                    cv.fillPoly(binary,[contours[j]],0)
            

            contours_info_sorted = sorted(contours_info, key = lambda distance: distance['distance'], reverse = False);
            for k in range(len(contours_info_sorted)):
                y_max = contours_info_sorted[k].get('y_max')
                if y_max > 470:
                    m_idx = k
                    break
            for k in range(len(contours_info_sorted)):
                if k != m_idx:
                    cv.fillPoly(binary, [contours_info_sorted[k].get('contour')], 0)
            if m_idx != -1:
                contour = contours_info_sorted[m_idx].get('contour')
        
        return binary,m_idx,contour

    def count_contours(self,img,threshold):
        contours_count = 0
        contours = cv.findContours(img, cv.RETR_EXTERNAL,cv.CHAIN_APPROX_SIMPLE)
        contours = contours[0]
        for item in contours:
            if cv.contourArea(item) > threshold:
                contours_count += 1
        return contours_count

    def method1(self,im,original_img,cnt_target, cnt_corss):
        new_img = cv.bitwise_not(im)
        new_img1 = new_img.copy()
        height,width = new_img.shape[:2]
        flag_cross = 0

        contours_count = self.count_contours(new_img,1000)
        if cnt_target == 1 and cnt_corss == 0:
            new_img1 = new_img1[int(height/2):height,0:width]           
            contours_count1 = self.count_contours(new_img1,1000)
            if contours_count > 1 or contours_count == 1 and contours_count1 > 2:
                flag_cross = 1
        elif cnt_target == 2 and cnt_corss == 0:
            new_img1 = new_img1[int(height/2):height,0:width]           
            contours_count1 = self.count_contours(new_img1,1000)
            if contours_count > 1 or contours_count == 1 and contours_count1 > 2:
                flag_cross = 1
            #if contours_count > 2:
                #flag_cross = 1qq
        else:
            if contours_count > 1:
                flag_cross = 1

        if flag_cross:
            contours = cv.findContours(im, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
            are = [cv.contourArea(i) for i in contours[0]]
            are = np.array(are)
            index = np.argmax(are)
            img = np.zeros(im.shape)
            im1 = cv.drawContours(img, contours[0], index, 255, -1)
            im = cv.morphologyEx(img, cv.MORPH_CLOSE, cv.getStructuringElement(cv.MORPH_RECT, (3, 3)))
        # cv.imshow('binary1', im1)
            x_acc = np.sum(im, axis=0)
            y_acc = np.sum(im, axis=1)
            x_diff = np.diff(x_acc)
            y_diff = np.diff(y_acc)
        # print(x_diff,y_diff)
            x_index1 = np.argmax(y_diff)
            x_index2 = np.argmin(y_diff)
            x_center = (x_index1 + x_index2) // 2
            y_index1 = np.argmax(x_diff)
            y_index2 = np.argmin(x_diff)
            y_center = (y_index1 + y_index2) // 2
            print("point:",(y_center,x_center))
            original_img = cv.circle(original_img, (y_center, x_center), 10, (0, 0, 255), -1)
            original_img = cv.cvtColor(original_img, cv.COLOR_BGR2RGB)
            #cv.imshow('original_img', original_img)
            #cv.imshow('binary', im)
            #cv.waitKey(0)
            #print(x_center,y_center)
            return (y_center, x_center),original_img,im
        else:
            #cv.imshow('original_img', original_img)
            #cv.imshow('binary', im)
            #cv.waitKey(0)
            return (0,0),original_img,im

    def enhance_brightness(self,image, alpha, beta):

        img_float = image.astype(float)
        enhanced_img = img_float * alpha + beta
        enhanced_img = np.clip(enhanced_img, 0, 255)
        enhanced_img = enhanced_img.astype(np.uint8)
        return enhanced_img

    def follow_white_line(self,rgb_img,hsv_msg):
        height, width = rgb_img.shape[:2]   
        img = rgb_img.copy()         
        #img = cv.convertScaleAbs(img, alpha, beta)
        #img= self.adapt_light(img)
        img[0:int(height / 2), 0:width] = 0
        hsv_img = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        lower = np.array(hsv_msg[0], dtype="uint8")
        upper = np.array(hsv_msg[1], dtype="uint8")
        # Create a mask based on a specific color range
        mask = cv.inRange(hsv_img, lower, upper)
        color_mask = cv.bitwise_and(hsv_img, hsv_img, mask=mask)
        #可改成cv.bitwise_and(hsv_img,mask)
        # Convert the image to grayscale
        gray_img = cv.cvtColor(color_mask, cv.COLOR_RGB2GRAY)
        # Get structure elements of different shapes
        kernel = cv.getStructuringElement(cv.MORPH_RECT, (5, 5))
        # Morphological closed operation
        gray_img = cv.morphologyEx(gray_img, cv.MORPH_CLOSE, kernel)
        # Image binarization operation
        src = cv.medianBlur(gray_img, 3)
        mean_intensity = cv.mean(src)[0]
        #print(mean_intensity)
        
        #if(mean_intensity >2):
            #result = self.adaptive_light(src)
        #else:
        result = src
        #ret, binary = cv.threshold(result, 10, 255, cv.THRESH_BINARY)
        ret, binary = cv.threshold(result, -1, 255, cv.THRESH_OTSU)
        return binary


    def line_follow(self, rgb_img, hsv_msg, flag,cnt_target, cnt_corss, finsh_flag, alpha,beta):  
        img = rgb_img.copy()
       
        binary = self.follow_white_line(img,hsv_msg)
               
        m_idx = -1
        binary,m_idx,contour = self.remove(binary,cnt_target,cnt_corss)
        (self.Center_x,self.Center_y), original_img, binary = self.method1(binary,rgb_img,cnt_target, cnt_corss)
        if flag != -1:
            flag -= 1
            if stop(flag,cnt_target,cnt_corss) == 1 :
                flag = 0
            self.nigger = 1
            if self.Center_x == 0 and self.Center_y == 0:
                return rgb_img, binary, (320,240,self.nigger),flag
            else:
                return rgb_img, binary, (self.Center_x, self.Center_y, self.nigger),flag

        if self.Center_x != 0 and self.Center_y != 0:
            if cnt_target == 1 and cnt_corss == 0:
                flag = 3
            elif cnt_target == 4 and (cnt_corss == 0 or cnt_corss == 6 or cnt_corss == 9 or cnt_corss == 10):
                flag = 0
            elif cnt_target == 4 and (cnt_corss == 4 or cnt_corss == 3):
                flag = 3
            elif cnt_target == 4 and cnt_corss == 1:
                flag = 5
            elif cnt_target == 4 and cnt_corss == 4:
                flag=3
            elif cnt_target == 5 and (cnt_corss == 3  or cnt_corss == 2 or cnt_corss == 4):
                flag=0
            else:
                flag = 1
        if m_idx != -1:
            front_x = 320
            front_y = 240
            max_rect = cv.minAreaRect(contour)
            max_box = cv.boxPoints(max_rect)
            max_box = np.int0(max_box)
            box = cv.boxPoints(max_rect)
            box = np.int0(box)
            (center_x, center_y), center_radius = cv.minEnclosingCircle(max_box)
            center_x = int(center_x)
            center_y = int(center_y)
            center_distance = []
            high_point = []
            cnt = contour
            cntx = cnt[:, 0, 0]
            cnty = cnt[:, 0, 1]
            m_i = np.argmax(cnty)
            m_i_r = np.argmin(cnty)
            indexes = [i for i, x in enumerate(cnty) if np.any(x == cnty[m_i])]
            indexes_high = [i for i, x in enumerate(cnty) if np.any(x == cnty[m_i_r])]
            m_i_l = 0
            for item in indexes:
                center_distance.append(abs(cntx[item]-center_x))
            m_i = indexes[np.argmin(center_distance)]
            for item in indexes_high:
                high_point.append(cntx[item])
            m_i_r = indexes_high[np.argmax(high_point)]
            cv.circle(rgb_img, (cntx[m_i_r],cnty[m_i_r]), 3, (255, 0, 255), -1)
            if len(cnty) > m_i+1:
                if cnty[m_i+1] == cnty[m_i]:
                    m_i_l = m_i+1              
                else:
                    m_i_l = m_i
                    m_i -= 1
            rear_x = (cntx[m_i] + cntx[m_i_l]) / 2
            rear_y = (cnty[m_i] + cnty[m_i_l]) / 2
            cv.circle(rgb_img, (int(rear_x),int(rear_y)), 3, (0, 0, 255), -1)
            if (cnt_target == 1 and cnt_corss == 1) or (cnt_target == 2 and cnt_corss == 1 and finsh_flag == 1) or flag != -1:
                return rgb_img, binary, (rear_x, rear_y, self.nigger), flag
            #找到最底下两点的的终点
            thre = (cntx[m_i_l] - cntx[m_i]) * 4/5
            thre_min = thre / 3
            m_i = 0
            while m_i <= m_i_l:
                #print(m_i,cntx[m_i],cnty[m_i])
                #print(m_i_l,cntx[m_i_l],cnty[m_i_l])
                if cnty[m_i] == cnty[m_i_r]:
                    if cntx[m_i_r] - cntx[m_i] <= thre and cntx[m_i_r] - cntx[m_i] > thre_min:
                        front_x = (cntx[m_i] + cntx[m_i_r]) / 2
                        front_y = (cnty[m_i] + cnty[m_i_r]) / 2
                        cv.circle(rgb_img, (int(front_x),int(front_y)), 3, (0, 0, 255), -1)
                        break
                    else:
                        m_i += 1
                        m_i_r -= 1
                elif cnty[m_i] > cnty[m_i_r]:
                    m_i_r -= 1
                else:
                    m_i += 1
            self.nigger = 1
            self.Center_x= front_x/2 + rear_x/2
            #print(self.Center_x)
            self.Center_y= front_y/2 + rear_y/2
        else:
            self.Center_x = 320
            self.Center_y = 240
            self.nigger = 0
        return rgb_img, binary, (self.Center_x, self.Center_y, self.nigger), flag
  
