#!/usr/bin/env python3

import rospy
import cv2
import json
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2.aruco as aruco

class ImageConverterPub:

    def __init__(self):
        self.id_pub = rospy.Publisher("/aruco_ID", String, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback)
        self.odom_sub= rospy.Subscriber("/odom",Odometry,self.odom_callback)
        self.waypoints = []

    def odom_callback(self,data):
        self.odom_data=data

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        markers_img, ids_list = self.detect_aruco(cv_image)

        if ids_list is not None:
            ids_str = ' '.join(str(e) for e in ids_list)
            self.id_pub.publish(ids_str)
            cv2.imshow("detected",markers_img)


       
        cv2.imshow("image window",cv_image)
        cv2.waitKey(3)

    def detect_aruco(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_250)
        parameters = aruco.DetectorParameters()
        # print("aruco marker detected")
        detector= aruco.ArucoDetector(aruco_dict,parameters)
        corners, ids, rejected = detector.detectMarkers(gray)
        print(ids)
        if ids is not None:
            for id in ids:
                print("entry added")
                self.waypoints.append({
                    'aruco_id': id[0],
                    'position': {
                        'x': self.odom_data.pose.pose.position.x,
                        'y': self.odom_data.pose.pose.position.y,
                        'z': self.odom_data.pose.pose.position.z
                    }
                })
            self.save_waypoints_to_json()
        output = aruco.drawDetectedMarkers(gray, corners, ids)
        return output, ids
    
    def save_waypoints_to_json(self):
        with open('waypoints.json', 'w') as json_file:
            json.dump(self.waypoints, json_file, indent=4)

def main():
    print("Initializing ROS-node")
    rospy.init_node('detect_markers', anonymous=True)
    print("Bring the ArUco markers in front of the camera")
    ic = ImageConverterPub()
    rospy.spin()

if __name__ == '__main__':
    main()
