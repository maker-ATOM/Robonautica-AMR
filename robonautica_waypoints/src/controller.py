#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseActionGoal
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose

import math
from tf import transformations

current_robot_pose = Pose()
robot_yaw = 0

def odom_callback(odom_data):
    global current_robot_pose
    current_robot_pose = odom_data.pose.pose
    global robot_yaw
    r, p, robot_yaw = transformations.euler_from_quaternion([0,0,current_robot_pose.orientation.z,current_robot_pose.orientation.w])
    r, p

if __name__ == '__main__':

    # id, x, y, yaw
    waypoints = [(1, 0.5, 0.1, 0.1),
                 (2, 1.0, 0.2, 0.2),
                 (3, 1.5, 0.3, 0.3)]
    distance_threshold = 0.12
    angular_threshold = 0.3
    goal_index = 0

    id_index = 0
    x_index = 1
    y_index = 2
    yaw_index = 3



    try:
    # -------------------------------------------------------------------#

        rospy.init_node("controller")

        pub = rospy.Publisher("/move_base/goal", MoveBaseActionGoal, queue_size = 10)
        rospy.Subscriber("/odom", Odometry, odom_callback)
        
        next_goal = MoveBaseActionGoal()
        next_goal.goal.target_pose.header.frame_id = "map"
        next_goal.goal.target_pose.pose.position.x = waypoints[goal_index][x_index]
        next_goal.goal.target_pose.pose.position.y = waypoints[goal_index][y_index]

        x, y, z, w = transformations.quaternion_from_euler(0,0,waypoints[goal_index][yaw_index])
 
        next_goal.goal.target_pose.pose.orientation.z = z
        next_goal.goal.target_pose.pose.orientation.w = w


        rate = rospy.Rate(50)

    # -------------------------------------------------------------------#

        while not rospy.is_shutdown():
            
            # should be within try and execpt 


            dx = waypoints[goal_index][x_index] - current_robot_pose.position.x
            dy = waypoints[goal_index][y_index] - current_robot_pose.position.y
            linear_error = math.sqrt(dx**2 + dy**2)

            angular_error = waypoints[goal_index][yaw_index] - robot_yaw

            print(f"Linear Error: {linear_error:.2f}, Angular Error: {angular_error:.2f}")

            if linear_error < distance_threshold and angular_error < angular_threshold:
                rospy.loginfo(f"“The UGV is at ({waypoints[goal_index][x_index]}, {waypoints[goal_index][y_index]}, {waypoints[goal_index][yaw_index]}) and has waypoint {waypoints[goal_index][id_index]}.")
                
                # wait for 15 seconds

                goal_index += 1
                if goal_index > len(waypoints)-1:
                    rospy.loginfo(f"End of waypoints")

                    # print finish time

                    exit()
                next_goal.goal.target_pose.pose.position.x = waypoints[goal_index][x_index]
                next_goal.goal.target_pose.pose.position.y = waypoints[goal_index][y_index]

                x, y, z, w = transformations.quaternion_from_euler(0,0,waypoints[goal_index][yaw_index])

                next_goal.goal.target_pose.pose.orientation.z = z
                next_goal.goal.target_pose.pose.orientation.w = w
                rospy.loginfo(f"Robot reached goal position, updating with next coordinates")

            rospy.sleep(1)
            pub.publish(next_goal)

            # rospy.loginfo(f"Goal pose {next_goal.goal.target_pose.pose}")
            rate.sleep()


    # -------------------------------------------------------------------#

        rospy.spin()

    except rospy.ROSInterruptException:
        pass