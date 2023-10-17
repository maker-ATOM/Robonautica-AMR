# This is an old /inactive file,Kindly Ignore




#!/usr/bin/env python3
import numpy as np
import rospy
import cv2
import json
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2.aruco as aruco
import imutils
import math
import os


class ImageConverterPub:

    def __init__(self):
        self.id_pub = rospy.Publisher("/aruco_ID", String, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.waypoints = []
        self.processed_ids = set()
        self.marker_x_buffer = []
        self.marker_y_buffer = []
        self.marker_distance_buffer = []

    def odom_callback(self, data):
        self.odom_data = data

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgra8")
        except CvBridgeError as e:
            print(e)

        markers_img, ids_list, corners = self.detect_aruco(cv_image)

        if ids_list is not None:
            ids_str = ' '.join(str(e) for e in ids_list)
            self.id_pub.publish(ids_str)
            cv2.imshow("detected", markers_img)
            self.process_markers(corners,ids_list)

        cv2.imshow("image window", cv_image)
        cv2.waitKey(3)

    def detect_aruco(self, img):
        amg = imutils.resize(img, 500)
        gray = cv2.cvtColor(amg, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
        Parameters = aruco.DetectorParameters_create()
        corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=Parameters)

        output = aruco.drawDetectedMarkers(gray, corners, ids)
        return output, ids, corners

    def process_markers(self, corners, ids_list):
        if self.odom_data is not None and corners is not None:
            x, y, z = round(self.odom_data.pose.pose.position.x, 3), round(self.odom_data.pose.pose.position.y, 3), round(self.odom_data.pose.pose.position.z, 3)
            for i, corner_set in enumerate(corners):
                marker_x = (corner_set[0][0] + corner_set[0][2]) / 2
                marker_y = (corner_set[0][1] + corner_set[0][3]) / 2

                marker_x[0], marker_y[0] = round(marker_x[0], 3), round(marker_y[0], 3)

                self.marker_x_buffer.append(marker_x[0])
                self.marker_y_buffer.append(marker_y[0])

                if len(self.marker_x_buffer) >= 10 and len(self.marker_y_buffer) >= 10:
                    # Calculate the average of the last 10 values of marker_x and marker_y
                    avg_marker_x = np.mean(self.marker_x_buffer[-10:])
                    avg_marker_y = np.mean(self.marker_y_buffer[-10:])
                    # Calculate marker_distance
                    marker_distance = math.sqrt((avg_marker_x - x) ** 2 + (avg_marker_y - y) ** 2 + z ** 2)
                    print(marker_distance)
                    aruco_id = int(ids_list[i][0])
                    
                    self.marker_distance_buffer.append(marker_distance)

                    if len(self.marker_distance_buffer) >= 5:
                        # Calculate the average of the last 5 marker distances
                        avg_marker_distance = np.mean(self.marker_distance_buffer[-5:])
                        print("Average Marker Distance (Last 5):", avg_marker_distance)

                    if aruco_id not in self.processed_ids and 1350 < avg_marker_distance < 1390:  # Set your desired threshold
                        print("Waypoint added for ArUco ID:", aruco_id)
                        self.waypoints.append({
                            'aruco_id': aruco_id,  # Add the ArUco ID to the waypoint entry
                            'position': {
                                'x': float(x),
                                'y': float(y),
                                'z': float(z)
                            }
                        })
                        self.processed_ids.add(aruco_id)
                        self.save_waypoints_to_json()
                        self.marker_x_buffer = []  # Clear the buffer after adding a waypoint
                        self.marker_y_buffer = []
                        self.marker_distance_buffer=[]

                    


                    
    def save_waypoints_to_json(self):
        with open('waypoints.json', 'w') as json_file:
            json.dump(self.waypoints, json_file, indent=4)
            print("Waypoints saved to JSON")

def main():
    print("Initializing ROS-node")
    rospy.init_node('detect_markers', anonymous=True)
    print("Bring the ArUco markers in front of the camera")
    ic = ImageConverterPub()
    rospy.spin()

if __name__ == '__main__':
    main()
