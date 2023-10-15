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
   
        angle = math.radians(20)

        obstacle_state = Bool()
        obstacle_state.data = False

        distance_threshold = 0.8

        rate = rospy.Rate(50)

    # -------------------------------------------------------------------#

        while not rospy.is_shutdown():
            
            try:

                # print(len(lidar_data.ranges))
                # print(lidar_data.angle_increment)

                l1 = lidar_data.ranges[120:140]
                
                # l1 = lidar_data.ranges[:int((angle / 2)/lidar_data.angle_increment)]
                l2 = lidar_data.ranges[-int((angle / 2)/lidar_data.angle_increment):-1]
                avg = sum(l1) / len(l1)
                # print(avg)
                if avg > distance_threshold:
                    obstacle_state.data = False
                else:
                    obstacle_state.data = True

            except:
                print("yet")
            pub.publish(obstacle_state)

            rate.sleep()


    # -------------------------------------------------------------------#

        rospy.spin()

    except rospy.ROSInterruptException:
        pass