#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan

import math

lidar_data = 9

def lidar_callback(msg):
    global lidar_data
    lidar_data = msg


if __name__ == '__main__':

    try:
    # -------------------------------------------------------------------#

        rospy.init_node("controller")

        pub = rospy.Publisher("/obstacle_state", Bool, queue_size = 10)
        rospy.Subscriber("/scan", LaserScan, lidar_callback)
   
        angle = math.radians(40)

        obstacle_state = Bool()
        obstacle_state.data = False

        distance_threshold = 0.8

        rate = rospy.Rate(50)

    # -------------------------------------------------------------------#

        while not rospy.is_shutdown():
            
            try:
                
                l1 = lidar_data.ranges[:int((angle / 2)/lidar_data.angle_increment)]
                l2 = lidar_data.ranges[-int((angle / 2)/lidar_data.angle_increment):-1]
                avg = sum(l1 + l2) / len(l1 + l2)
                if avg > distance_threshold:
                    obstacle_state.data = False
                else:
                    obstacle_state.data = True

                print(obstacle_state.data)
            except:
                print("yet")
            pub.publish(obstacle_state)

            rate.sleep()


    # -------------------------------------------------------------------#

        rospy.spin()

    except rospy.ROSInterruptException:
        pass