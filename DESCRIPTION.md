## Task 1.1 : Mapping

For mapping purpose we have utilized gmapping package, which requires two primary things odometry data and lidar data. The lidar was readily available by the Lidar sensor but the odometry data was not provided by the robot. We had to figure out on our own.

The `gmapping.launch` files launches the gmapping node with the set parameters.

For odometry we initially had a approach to figure out the relative displacement of the robot from the initially position using robot wheel speed integrated with respect to time for each iteration of loop, to make the generated odometry data more reliable it can fused with inertial measurement unit.

Ultimately we used `icp_odometry` form `rtabmap_odom` package which uses lidar data to find the odometry data.

## Task 1.2 Waypoint generation

We used `cv2.aruco` library for marker detection and tweaked its parameters for correct marker detection. Also we used its corners for marker distance for marker orientation. Then created a waypoint list where all the aruco id and the position of the robot where the aruco is detected is appended and saved in json file as a means of waypoints.

`set_2.py` is the latest file used for aurco marker detection and and waypoint storage.

## Task 2,1 Autonomous Navigation

For autonomous navigation we utilized `move_base` entity within the navigation stack, so the `navigation.launch` files launches the `amcl` and `move_base` nodes, the parameters for `move_base` to generate the local and global cost map as well as to perform path planning and obstacle avoidance and stored within param folder.

This navigation stack in conjugation with `controller.py` script which provides the next goal pose once current is reached within the generated list stored by the `set_2.py` node completes the autonomous navigation task.