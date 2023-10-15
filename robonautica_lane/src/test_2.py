#!/usr/bin/env python3
import roslib
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

def calculate_line_equation(x1, y1, x2, y2):
    # Calculate slope (m) and y-intercept (c) of a line given two points
    m = (y2 - y1) / (x2 - x1)
    c = y1 - m * x1
    return m, c

def print_line_equation(m, c, lane_type):
    # Print the line equation in the form y = mx + c
    rospy.loginfo(f"{lane_type} Lane Equation: y = {m}x + {c}")

def callback(data):
    # rospy.loginfo("ciao")
    # Convert image to cv2 standard format
    img = bridge.imgmsg_to_cv2(data)

    # Start time
    start_time = cv2.getTickCount()

    # Gaussian Filter to remove noise
    img = cv2.medianBlur(img, 5)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    

    # Print img.shape = (200, 350, 3)
    rows, cols, channels = img.shape

    # ROI
    roi_mask = np.zeros(img.shape, dtype=np.uint8)
    roi_mask[10:rows, 0:cols] = 255
    street = cv2.bitwise_and(img, roi_mask)

    yellow_roi_mask = np.zeros(gray.shape, dtype=np.uint8)
    yellow_roi_mask[150:rows, 0:cols] = 255

    # Define range of yellow color in HSV
    hsv = cv2.cvtColor(street, cv2.COLOR_BGR2HSV)
    cv2.imshow("duhs",hsv)
    cv2.waitKey(3)

    lower_yellow = np.array([20, 100, 100])  # Adjust this range as needed
    upper_yellow = np.array([40, 255, 255])  # Adjust this range as needed

    yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    yellow_mask = cv2.erode(yellow_mask, None, iterations=2)
    yellow_mask = cv2.dilate(yellow_mask, None, iterations=2)

    # Mask AND original img
    yellow_hsv_thresh = cv2.bitwise_and(street, street, mask=yellow_mask)

    # Canny Edge Detection
    yellow_edges = cv2.Canny(yellow_hsv_thresh, 100, 200)
    yellow_edges = cv2.bitwise_and(yellow_edges, yellow_roi_mask)

    # Standard Hough Transform
    yellow_lines = cv2.HoughLines(yellow_edges, 0.8, np.pi / 180, 30)

    xm = cols / 2
    ym = rows

    # Draw yellow lanes
    left_lane_x = []
    right_lane_x = []

    if yellow_lines is not None:
        yellow_lines = np.array(yellow_lines[0])
        for rho, theta in yellow_lines:
            a = np.cos(theta)
            b = np.sin(theta)
            x0, y0 = a * rho, b * rho
            y3 = 140
            x3 = int(x0 + ((y0 - y3) * np.sin(theta) / np.cos(theta)))
            
            if x3 < xm:
                left_lane_x.append(x3)
                pt1 = (int(x0 + 1000 * (-b)), int(y0 + 1000 * (a)))
                pt2 = (int(x0 - 1000 * (-b)), int(y0 - 1000 * (a)))
                cv2.line(img, pt1, pt2, (0, 255, 0), 2)
            else:
                right_lane_x.append(x3)
                pt1 = (int(x0 + 1000 * (-b)), int(y0 + 1000 * (a)))
                pt2 = (int(x0 - 1000 * (-b)), int(y0 - 1000 * (a)))
                cv2.line(img, pt1, pt2, (0, 0, 255), 2)

    if left_lane_x:
        xmin_left = min(left_lane_x)
        kl = int(np.sqrt(((xmin_left - xm) * (xmin_left - xm)) + ((y3 - ym) * (y3 - ym))))
        m, c = calculate_line_equation(xm, ym, xmin_left, y3)
        print_line_equation(m, c, "Left")
    else:
        kl = 0

    if right_lane_x:
        xmax_right = max(right_lane_x)
        kr = int(np.sqrt(((xmax_right - xm) * (xmax_right - xm)) + ((y3 - ym) * (y3 - ym))))
        m, c = calculate_line_equation(xm, ym, xmax_right, y3)
        print_line_equation(m, c, "Right")
    else:
        kr = 0

    # End time
    end_time = cv2.getTickCount()

    time_count = (end_time - start_time) / cv2.getTickFrequency()

    # rospy.loginfo("Time Count: %f" % time_count)

def lane_detection():
    rospy.init_node('lane-detection', anonymous=True)
    rospy.Subscriber("/camera/color/image_raw", Image, callback, queue_size=1, buff_size=2**24)
    try:
        rospy.loginfo("Entering ROS Spin")
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    try:
        lane_detection()
    except rospy.ROSInterruptException:
        pass
