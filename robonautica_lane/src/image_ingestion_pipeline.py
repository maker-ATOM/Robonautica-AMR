import cv2
import numpy as np
import matplotlib.pyplot as plt
import glob
import os
from moviepy.editor import *

class laneDetection:
    def detectlanes(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HLS)
        
         #Yellow color mask
        lower_threshold = np.uint8([10, 0, 100])
        upper_threshold = np.uint8([40, 255, 255])
        yellow_mask = cv2.inRange(hsv, lower_threshold, upper_threshold)
        masked_image = cv2.bitwise_and(image, image, mask = yellow_mask)

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
        self.showimgs(res)

    def draw_lines(self, image, lines, color = [255, 0, 0], thickness = 2):
        """
        Draw lines onto the input image.
            Parameters:
                image: An np.array compatible with plt.imshow.
                lines: The lines we want to draw.
                color (Default = red): Line color.
                thickness (Default = 2): Line thickness.
        """
        image = np.copy(image)
        for line in lines:
            for x1,y1,x2,y2 in line:
                cv2.line(image, (x1, y1), (x2, y2), color, thickness)
        return image
    
    def showimgs(self, image):
        cv2.imshow('',image)
        cv2.waitKey(0)

if __name__ == "__main__":
    test = laneDetection()
    image = plt.imread('test_images\\robonauticasnap.jpg')
    test.detectlanes(image)