import cv2
import numpy as np
import matplotlib.pyplot as plt
import glob
import os
from moviepy.editor import *

class laneDetection:
    def detectlanes(self, image):
        image = self.clipimage(image)
        
        hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HLS)
        
         #Yellow color mask

        lower_threshold = np.uint8([10, 0, 100])
        upper_threshold = np.uint8([40, 255, 255])
        yellow_mask = cv2.inRange(hsv, lower_threshold, upper_threshold)
        masked_image = cv2.bitwise_and(image, image, mask = yellow_mask)
        self.showimgs(masked_image)
        gray_image = cv2.cvtColor(masked_image, cv2.COLOR_RGB2GRAY)
        kernel_size = 13
        blur_image = cv2.GaussianBlur(gray_image, (kernel_size, kernel_size), 0)
        low_threshold = 50 
        high_threshold = 150
        edge_detected = cv2.Canny(blur_image, low_threshold, high_threshold)
        

        rho = 1              #Distance resolution of the accumulator in pixels.
        theta = np.pi/180    #Angle resolution of the accumulator in radians.
        threshold = 20       #Only lines that are greater than threshold will be returned.
        minLineLength = 20   #Line segments shorter than that are rejected.
        maxLineGap = 300     #Maximum allowed gap between points on the same line to link them
        hough_lines = cv2.HoughLinesP(edge_detected, rho = rho, theta = theta, threshold = threshold,
                           minLineLength = minLineLength, maxLineGap = maxLineGap)
        res = self.draw_lines(image, hough_lines)
        
        
        lanelines = self.lane_lines(image, hough_lines)
        filteredlines = []
        if lanelines is not None:
            filteredlines.append([(lanelines[0][0][0], lanelines[0][0][1], lanelines[0][1][0], lanelines[0][1][1]), (lanelines[1][0][0], lanelines[1][0][1], lanelines[1][1][0], lanelines[1][1][1])])
            # print(filteredlines)
            self.showimgs(self.draw_lines(image, filteredlines))
            x1,y1,x2,y2 = filteredlines[0][0]
            # print((y2-y1)/(x2-x1))
            return (y2-y1)/(x2-x1)
        else:
            return None
        # return lanelines
        #print(lanelines)
        #viz = self.draw_lane_lines(image, lanelines)
        #self.showimgs(viz)
        

    def clipimage(self, image):
        x, y, z = image.shape
        x1, y1 = int(0.3*x), 0  # Top-left corner
        x2, y2 = int(x), y  # Bottom-right corner

        # Crop the ROI from the image
        return image[y1:y2, x1:x2, :]


    def draw_lines(self, image, lines, color = [255, 0, 0], thickness = 2):
        
        image = np.copy(image)
        for line in lines:
            for x1,y1,x2,y2 in line:
                cv2.line(image, (x1, y1), (x2, y2), color, thickness)
        return image
    
    def showimgs(self, image):
        cv2.imshow('',image)
        # cv2.waitKey(0)

    def average_slope_intercept(self, lines):
        
        left_lines    = [] #(slope, intercept)
        left_weights  = [] #(length,)
        right_lines   = [] #(slope, intercept)
        right_weights = [] #(length,)
        
        for line in lines:
            for x1, y1, x2, y2 in line:
                if x1 == x2:
                    continue
                slope = (y2 - y1) / (x2 - x1)
                intercept = y1 - (slope * x1)
                length = np.sqrt(((y2 - y1) ** 2) + ((x2 - x1) ** 2))
                if slope < 0:
                    left_lines.append((slope, intercept))
                    left_weights.append((length))
                else:
                    right_lines.append((slope, intercept))
                    right_weights.append((length))
        left_lane  = np.dot(left_weights,  left_lines) / np.sum(left_weights)  if len(left_weights) > 0 else None
        right_lane = np.dot(right_weights, right_lines) / np.sum(right_weights) if len(right_weights) > 0 else None
        return left_lane, right_lane
    
    def pixel_points(self, y1, y2, line):
        
        if line is None:
            return None
        slope, intercept = line
        x1 = int((y1 - intercept)/slope)
        x2 = int((y2 - intercept)/slope)
        y1 = int(y1)
        y2 = int(y2)
        return ((x1, y1), (x2, y2))
    
    def lane_lines(self, image, lines):
        
        left_lane, right_lane = self.average_slope_intercept(lines)
        y1 = image.shape[0]
        y2 = y1 * 0.6
        left_line  = self.pixel_points(y1, y2, left_lane)
        right_line = self.pixel_points(y1, y2, right_lane)
        return left_line, right_line
    
    def draw_lane_lines(image, lines, color=[255, 0, 0], thickness=12):
        
        line_image = np.zeros_like(image)
        for line in lines:
            if line is not None:
                cv2.line(line_image, *line,  color, thickness)
        return cv2.addWeighted(image, 1.0, line_image, 1.0, 0.0)

if __name__ == "__main__":
    test = laneDetection()
    image = plt.imread('test_images\\robonauticasnap6.jpg')
    test.detectlanes(image)