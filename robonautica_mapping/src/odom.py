#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseActionGoal
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

import math
from tf import transformations

robot_vel = 0

def cmd_vel_callback(data):
    global robot_vel
    robot_vel = data
    # print(robot_vel)

if __name__ == '__main__':

    try:
    # -------------------------------------------------------------------#

        rospy.init_node("odom_pub")

        rospy.Subscriber("/cmd_vel", Twist, cmd_vel_callback)
        pub = rospy.Publisher("/odom_vel", Odometry, queue_size = 10)
        
        dt = 1/50
        theta = 0
        x = 0
        y = 0
        prev_time = 0
        current_time = 0

        rate = rospy.Rate(50)

    # -------------------------------------------------------------------#

        while not rospy.is_shutdown():
            prev_time = current_time

            current_time = rospy.Time.now().nsecs

            if current_time > prev_time:
                dt = (current_time - prev_time) / 900000000

            # print(current_time, prev_time, dt)

            try:
                theta += robot_vel.angular.z * dt
                qx, qy, qz, qw = transformations.quaternion_from_euler(0,0,theta)
                x += robot_vel.linear.x * math.cos(theta) * dt
                y += robot_vel.linear.y * math.sin(theta) * dt
                print(f"{x:.2f}, {y:.2f}, {qz:.2f}, {qw:.2f}")
                odom_data = Odometry()

                odom_data.header.frame_id = "odom"
                odom_data.child_frame_id = "base_link"
                odom_data.pose.pose.position.x = x
                odom_data.pose.pose.position.y = y
                odom_data.pose.pose.orientation.z = qz
                odom_data.pose.pose.orientation.w = qw

                pub.publish(odom_data)
            except:
                print("yet to publisah")




            
 
            rate.sleep()

            


    # -------------------------------------------------------------------#

        rospy.spin()

    except rospy.ROSInterruptException:
        pass