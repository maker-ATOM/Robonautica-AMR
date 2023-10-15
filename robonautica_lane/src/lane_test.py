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

class LaneFollower:
    def __init__(self):
        rospy.init_node('lane_follower')
        print("node started")
        self.kp=0.1
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.obstacle_sub= rospy.Subscriber('/obstacle_state',Bool,self.obs_callback)
        self.twist = Twist()

    def obs_callback(self,val):
        try:
            # print(val)
            self.value=val
        except:
            print(" no data")    

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
        '''
        Perprorrocess the input image to detect lane lines.
            Parameters:
                image: list of image/s
        '''
        # print("before processing")
        imgProc = laneDetection()
        lanes = imgProc.detectlanes(image)
        print(lanes)
        if(lanes is not None):
            lanes_prev= lanes
        else:
            return (lanes_prev)    
        # error=slope_diff(lanes)
        # print(error)
        # cv2.imshow("lane Image",lanes[0])
        # cv2.waitKey(3)
        # print("after Processing")
        return (lanes)

    def calculate_cmd_vel(self, lanes):
        # print("a")
        staright=3
        turn =2.5
        threshold=-1.2
        linear_x=0.5
        error= threshold-lanes
        angular_z= self.kp*error
        print(self.value.data)
        if self.value.data== False:
            # print(linear_x)
            self.twist.linear.x= linear_x
            self.twist.angular.z=angular_z
        elif self.value.data==True:
            # print()
            self.twist.linear.x= 0
            self.twist.angular.z=-0.5
            self.cmd_vel_pub.publish(self.twist)
            rospy.sleep(turn)
            self.twist.linear.x= 1
            self.twist.angular.z=0
            self.cmd_vel_pub.publish(self.twist)
            rospy.sleep(staright)
            self.twist.linear.x= 0
            self.twist.angular.z=0.5
            self.cmd_vel_pub.publish(self.twist)
            rospy.sleep(turn)
            self.twist.linear.x= 1
            self.twist.angular.z=0
            self.cmd_vel_pub.publish(self.twist)
            rospy.sleep(staright)
            self.twist.linear.x= 0
            self.twist.angular.z=0.5
            self.cmd_vel_pub.publish(self.twist)
            rospy.sleep(turn)
            self.twist.linear.x= 1
            self.twist.angular.z=0
            self.cmd_vel_pub.publish(self.twist)
            rospy.sleep(staright)
            self.twist.linear.x= 0
            self.twist.angular.z=-0.5
            self.cmd_vel_pub.publish(self.twist)
            rospy.sleep(turn)


        # elif count==1 and:
        #     self.twist.linear.x= 0
        #     self.twist.angular.z=0.5
        # elif count==2:
        #     self.twist.linear.x= 0.5
        #     self.twist.angular.z= 0   
        # elif count==3:
        #     self.twist.linear.x= 0
        #     self.twist.angular.z= -0.5      

        return self.twist

        
    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()

    # def draw_lane_lines(self ,image, lines, color=[255, 0, 0], thickness=12):
    #     """
    #     Draw lines onto the input image.
    #         Parameters:
    #             image: The input test image.
    #             lines: The output lines from Hough Transform.
    #             color (Default = red): Line color.
    #             thickness (Default = 12): Line thickness. 
    #     """
    #     line_image = np.zeros_like(image)
    #     for line in lines:
    #         if line is not None:
    #             cv2.line(line_image, *line,  color, thickness)
    #     return cv2.addWeighted(image, 1.0, line_image, 1.0, 0.0)

if __name__ == '__main__':
    try:
        lane_follower = LaneFollower()
        lane_follower.run()
    except rospy.ROSInterruptException:
        pass
