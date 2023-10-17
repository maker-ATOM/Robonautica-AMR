# This is an old /inactive file,Kindly Ignore


#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

import math

bot_state = 0


def callback(msg):
    global bot_state
    bot_state = msg.data
    print(bot_state)


if __name__ == '__main__':
    bot_state
    try:
    # -------------------------------------------------------------------#

        rospy.init_node("controller")

        pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
        rospy.Subscriber("/sign", Int32, callback)

        bot_speed = Twist()
        bot_speed.linear.x = 0
        bot_speed.angular.y = 0

        

        rate = rospy.Rate(50)

    # -------------------------------------------------------------------#

        while not rospy.is_shutdown():
            
            try:
                if bot_state == "straight":
                    bot_speed.linear.x = 0.2
                    bot_state.angular.x =0.0
                elif bot_state == "left":
                    bot_speed.linear.x = 0.0
                    bot_speed.angular.z = 0.5
                elif bot_state == "right":
                    bot_speed.linear.x = 0.0
                    bot_speed.angular.z = -0.5
                
            
            except:
                print("yet")
            # print(bot_speed)
            pub.publish(bot_speed)

            rate.sleep()


    # -------------------------------------------------------------------#

        rospy.spin()

    except rospy.ROSInterruptException:
        pass