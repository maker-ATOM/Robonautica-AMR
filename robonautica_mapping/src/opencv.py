#!/usr/bin/env python3

import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2.aruco as aruco

class ImageConverterPub:

    def __init__(self):
        self.image_pub = rospy.Publisher("/detected_markers", Image, queue_size=1)
        self.id_pub = rospy.Publisher("/aruco_ID", String, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        markers_img, ids_list = self.detect_aruco(cv_image)

        if ids_list is not None:
            ids_str = ' '.join(str(e) for e in ids_list)
            self.id_pub.publish(ids_str)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(markers_img, "bgr8"))
        except CvBridgeError as e:
            print(e)  
        cv2.imshow("image window",cv_image)
        cv2.waitKey(3)

    def detect_aruco(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_250)
        parameters = aruco.DetectorParameters()
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        output = aruco.drawDetectedMarkers(img.copy(), corners, ids)
        return output, ids

def main():
    print("Initializing ROS-node")
    rospy.init_node('detect_markers', anonymous=True)
    print("Bring the ArUco markers in front of the camera")
    ic = ImageConverterPub()
    rospy.spin()

if __name__ == '__main__':
    main()
