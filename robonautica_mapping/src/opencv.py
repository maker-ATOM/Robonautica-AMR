# This is an old /inactive file,Kindly Ignore


#!/usr/bin/env python3

import rospy
import cv2
import json
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2.aruco as aruco
import imutils

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
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgra8")
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
        amg=imutils.resize(img,500)
        gray = cv2.cvtColor(amg, cv2.COLOR_BGR2GRAY)
        # aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        aruco_dict=aruco.Dictionary_get(aruco.DICT_4X4_250)
        # parameters = aruco.DetectorParameters()
        Parameters= aruco.DetectorParameters_create()
        # print("aruco marker detected")
        # detector= aruco.ArucoDetector(aruco_dict,parameters)
        corners, ids, rejected = aruco.detectMarkers(gray,aruco_dict,parameters=Parameters)
        print(ids)
        if ids is not None:
            for id in ids:
                # print("entry added")
                x=float(round(self.odom_data.pose.pose.position.x,3))
                y=float(round(self.odom_data.pose.pose.position.y,3))
                z=float(round(self.odom_data.pose.pose.position.z,3))
                # print(x)

                self.waypoints.append({
                    'aruco_id': id[0],
                    'position': {
                        'x': float(x),
                        'y': float(y),
                        'z': float(z)
                    }
                })
            self.save_waypoints_to_json()
        output = aruco.drawDetectedMarkers(gray, corners, ids)
        return output, ids
    
    def save_waypoints_to_json(self):
        with open('waypoints.json', 'w') as json_file:
            json.dump(self.waypoints, json_file, indent=4)
            print("dumped")

def main():
    print("Initializing ROS-node")
    rospy.init_node('detect_markers', anonymous=True)
    print("Bring the ArUco markers in front of the camera")
    ic = ImageConverterPub()
    rospy.spin()

if __name__ == '__main__':
    main()
