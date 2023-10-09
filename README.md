## Task Completed

- [x] Spawn at the origin of the environment 
- [x] Teleoperate the robot and map the environment.

## Task pending

- [] Scan the ArUco markers and store the robot’s closest possible position (pose)
with respect to the marker when scanning the ArUco marker.
- [] Autonomously navigate to each ArUco marker
- [] Collision avoidance and control of the robot.


## Ideology

Initially

## Errors during Execution

## What next?

## Usage

Clone the repository




OUPUT after launcing gmapping
rosrun gmapping slam_gmapping scan:=scan
Warning: TF_REPEATED_DATA ignoring data with redundant timestamp for frame odom at time 2811.754000 according to authority unknown_publisher
         at line 278 in /tmp/binarydeb/ros-noetic-tf2-0.7.6/src/buffer_core.cpp
[ WARN] [1696841508.422509537, 2826.655000000]: MessageFilter [target=odom ]: Dropped 100.00% of messages so far. Please turn the [ros.gmapping.message_filter] rosconsole logger to DEBUG for more information.



rostopic echo /tf_static
transforms: 
  - 
    header: 
      seq: 0
      stamp: 
        secs: 2771
        nsecs: 937000000
      frame_id: "base_link"
    child_frame_id: "back_castor_1"
    transform: 
      translation: 
        x: -0.103994
        y: 0.000262
        z: 0.01653
      rotation: 
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0
  - 
    header: 
      seq: 0
      stamp: 
        secs: 2771
        nsecs: 937000000
      frame_id: "camera_depth_frame"
    child_frame_id: "camera_color_frame"
    transform: 
      translation: 
        x: 0.0
        y: 0.015
        z: 0.0
      rotation: 
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0
  - 
    header: 
      seq: 0
      stamp: 
        secs: 2771
        nsecs: 937000000
      frame_id: "camera_color_frame"
    child_frame_id: "camera_color_optical_frame"
    transform: 
      translation: 
        x: 0.0
        y: 0.0
        z: 0.0
      rotation: 
        x: 0.5
        y: -0.4999999999999999
        z: 0.5
        w: -0.5000000000000001
  - 
    header: 
      seq: 0
      stamp: 
        secs: 2771
        nsecs: 937000000
      frame_id: "camera_link"
    child_frame_id: "camera_depth_frame"
    transform: 
      translation: 
        x: 0.0
        y: 0.0
        z: 0.0
      rotation: 
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0
  - 
    header: 
      seq: 0
      stamp: 
        secs: 2771
        nsecs: 937000000
      frame_id: "camera_depth_frame"
    child_frame_id: "camera_depth_optical_frame"
    transform: 
      translation: 
        x: 0.0
        y: 0.0
        z: 0.0
      rotation: 
        x: 0.5
        y: -0.4999999999999999
        z: 0.5
        w: -0.5000000000000001
  - 
    header: 
      seq: 0
      stamp: 
        secs: 2771
        nsecs: 937000000
      frame_id: "realsense_1"
    child_frame_id: "camera_bottom_screw_frame"
    transform: 
      translation: 
        x: 0.011753067204287801
        y: -0.0004295385558066378
        z: -0.035091351352319555
      rotation: 
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0
  - 
    header: 
      seq: 0
      stamp: 
        secs: 2771
        nsecs: 937000000
      frame_id: "camera_depth_frame"
    child_frame_id: "camera_left_ir_frame"
    transform: 
      translation: 
        x: 0.0
        y: 0.0
        z: 0.0
      rotation: 
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0
  - 
    header: 
      seq: 0
      stamp: 
        secs: 2771
        nsecs: 937000000
      frame_id: "camera_left_ir_frame"
    child_frame_id: "camera_left_ir_optical_frame"
    transform: 
      translation: 
        x: 0.0
        y: 0.0
        z: 0.0
      rotation: 
        x: 0.5
        y: -0.4999999999999999
        z: 0.5
        w: -0.5000000000000001
  - 
    header: 
      seq: 0
      stamp: 
        secs: 2771
        nsecs: 937000000
      frame_id: "camera_bottom_screw_frame"
    child_frame_id: "camera_link"
    transform: 
      translation: 
        x: 0.0
        y: 0.0175
        z: 0.0125
      rotation: 
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0
  - 
    header: 
      seq: 0
      stamp: 
        secs: 2771
        nsecs: 937000000
      frame_id: "camera_depth_frame"
    child_frame_id: "camera_right_ir_frame"
    transform: 
      translation: 
        x: 0.0
        y: -0.05
        z: 0.0
      rotation: 
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0
  - 
    header: 
      seq: 0
      stamp: 
        secs: 2771
        nsecs: 937000000
      frame_id: "camera_right_ir_frame"
    child_frame_id: "camera_right_ir_optical_frame"
    transform: 
      translation: 
        x: 0.0
        y: 0.0
        z: 0.0
      rotation: 
        x: 0.5
        y: -0.4999999999999999
        z: 0.5
        w: -0.5000000000000001
  - 
    header: 
      seq: 0
      stamp: 
        secs: 2771
        nsecs: 937000000
      frame_id: "base_link"
    child_frame_id: "front_castor_1"
    transform: 
      translation: 
        x: 0.120061
        y: 0.000262
        z: 0.01653
      rotation: 
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0
  - 
    header: 
      seq: 0
      stamp: 
        secs: 2771
        nsecs: 937000000
      frame_id: "base_link"
    child_frame_id: "imu_1"
    transform: 
      translation: 
        x: 0.006515
        y: 4e-06
        z: 0.04506
      rotation: 
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0
  - 
    header: 
      seq: 0
      stamp: 
        secs: 2771
        nsecs: 937000000
      frame_id: "base_link"
    child_frame_id: "lidar_base_1"
    transform: 
      translation: 
        x: -0.067038
        y: 4e-06
        z: 0.14753
      rotation: 
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0
  - 
    header: 
      seq: 0
      stamp: 
        secs: 2771
        nsecs: 937000000
      frame_id: "lidar_base_1"
    child_frame_id: "lidar_1"
    transform: 
      translation: 
        x: 0.0353
        y: 0.0
        z: 0.047696
      rotation: 
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0
  - 
    header: 
      seq: 0
      stamp: 
        secs: 2771
        nsecs: 937000000
      frame_id: "base_link"
    child_frame_id: "realsense_1"
    transform: 
      translation: 
        x: -0.08369
        y: -1.8e-05
        z: 0.289948
      rotation: 
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0
---

