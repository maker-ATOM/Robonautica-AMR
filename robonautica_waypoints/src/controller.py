#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseActionGoal
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose

import math
import tf

current_robot_pose = Pose()

def odom_callback(odom_data):
    global current_robot_pose
    current_robot_pose = odom_data.pose.pose

if __name__ == '__main__':

    waypoints = [(4,1,0,1),(3,2,0,1),(1,1,0,1)]
    distance_threshold = 0.1
    angular_threshold = 0.05
    goal_index = 0

    try:
    # -------------------------------------------------------------------#

        rospy.init_node("controller")

        pub = rospy.Publisher("/move_base/goal", MoveBaseActionGoal, queue_size = 10)
        rospy.Subscriber("/odom", Odometry, odom_callback)
        
        next_goal = MoveBaseActionGoal()
        next_goal.goal.target_pose.header.frame_id = "map"
        next_goal.goal.target_pose.pose.position.x = waypoints[goal_index][0]
        next_goal.goal.target_pose.pose.position.y = waypoints[goal_index][1]
        next_goal.goal.target_pose.pose.orientation.z = waypoints[goal_index][2]
        next_goal.goal.target_pose.pose.orientation.w = waypoints[goal_index][3]


        rate = rospy.Rate(50)

    # -------------------------------------------------------------------#

        while not rospy.is_shutdown():
            
            # should be within try and execpt 
            if goal_index > len(waypoints):
                rospy.loginfo(f"End of waypoints")
                exit()

            dx = waypoints[goal_index][0] - current_robot_pose.position.x
            dy = waypoints[goal_index][1] - current_robot_pose.position.y
            linear_error = math.sqrt(dx**2 + dy**2)

            dz = waypoints[goal_index][2] - current_robot_pose.orientation.z
            dw = waypoints[goal_index][3] - current_robot_pose.orientation.w
            angular_error = math.sqrt(dz**2 + dw**2)

            # print(f"{linear_error:.2f}, {angular_error:.2f}")
            # print(f"{dy:.2f}, {dz:.2f}, { current_robot_pose.orientation.z:.2f}, { current_robot_pose.orientation.w:.2f}")

            if linear_error < distance_threshold and angular_error < angular_threshold:
                goal_index += 1
                next_goal.goal.target_pose.pose.position.x = waypoints[goal_index][0]
                next_goal.goal.target_pose.pose.position.y = waypoints[goal_index][1]
                next_goal.goal.target_pose.pose.orientation.z = waypoints[goal_index][2]
                next_goal.goal.target_pose.pose.orientation.w = waypoints[goal_index][3]
                rospy.loginfo(f"Robot reached goal position, updating with next coordinates")

            rospy.sleep(1)
            pub.publish(next_goal)

            # rospy.loginfo(f"Goal pose {next_goal.goal.target_pose.pose}")
            rate.sleep()


    # -------------------------------------------------------------------#

        rospy.spin()

    except rospy.ROSInterruptException:
        pass