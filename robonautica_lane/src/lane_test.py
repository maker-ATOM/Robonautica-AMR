#!/usr/bin/env python3
from controller import slope_diff
import rospy
import cv2
import numpy as np
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from image_ingestion_pipeline import laneDetection
from sensor_msgs.msg import LaserScan

class LaneFollower:
    def __init__(self):
        rospy.init_node('lane_follower')
        print("node started")
        self.kp=0.1
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.obstacle_sub= rospy.Subscriber('/obstacle_state',Bool,self.obs_callback)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.twist = Twist()

    def obs_callback(self,val):
        try:
            # print(val)
            self.value=val
        except:
            print(" no data")  

    def scan_callback(self, val):
        self.scan_data = val.ranges
        front=self.sum(self.scan_data[140:150])/20
        right=self.sum(self.scan_data[185:205])/20
        left= self.sum(self.scan_data[55:75])/20
        

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # Apply image processing to detect lanes
        processed_lane = self.process_image(cv_image)
        cv2.imshow("output lane",cv_image)
        cv2.waitKey(3)

        # count= 
        # Calculate control commands based on lane detection
        cmd_vel = self.calculate_cmd_vel(processed_lane)

        # Publish the control commands
        self.cmd_vel_pub.publish(cmd_vel)

    def process_image(self, image):
        # print("a")
        # Implement your lane detection algorithm here
        # You can use techniques like color filtering, edge detection, and Hough transform
        # and update this method to return the processed image
        
        # print("before processing")
        imgProc = laneDetection()
        lanes = imgProc.detectlanes(image)
        print(lanes)
        if(lanes is not None):
            lanes_prev= lanes
        else:
            return (lanes_prev)    

        return (lanes)

    def calculate_cmd_vel(self, lanes):
        staright=3
        turn =2.5
        threshold=-1.2
        linear_x=0.5
        side_threshold = 9.0
        ob_threshold = 9.5
        error= threshold-lanes
        angular_z= self.kp*error
        robot_state = 0
        print(self.value.data)
        if self.value.data== False:
            self.twist.linear.x= linear_x
            self.twist.angular.z=angular_z
            robot_state = 1
        elif self.value.data==True and robot_state == 1:
            if self.left > side_threshold and self.right > side_threshold:
                self.twist.linear.x= 0
                self.twist.angular.z=-0.5
                robot_state = 3
                self.cmd_vel_pub.publish(self.twist)
            elif self.left < ob_threshold and robot_state == 3:
                self.twist.linear.x= 1
                self.twist.angular.z=0
                robot_state = 4
                self.cmd_vel_pub.publish(self.twist)
            elif self.left > side_threshold and robot_state == 4:
                robot_state = 3
            else:
                robot_state = 0
                self.value.data = False
   

        return self.twist

        
    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == '__main__':
    try:
        lane_follower = LaneFollower()
        lane_follower.run()
    except rospy.ROSInterruptException:
        pass
