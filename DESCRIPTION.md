## Task 1.1 : Mapping

For mapping purpose we have utilized gmapping package, which requires two primary things odometry data and lidar data. The lidar was readily available by the Lidar sensor but the odometry data was not provided by the robot. We had to figure out on our own.

The `gmapping.launch` files launches the gmapping node with the set parameters.

For odometry we initially had a approach to figure out the relative displacement of the robot from the initially position using robot wheel speed integrated with respect to time for each iteration of loop, to make the generated odometry data more reliable it can fused with inertial measurement unit.

Ultimately we used `icp_odometry` form `rtabmap_odom` package which uses lidar data to find the odometry data.

## Task 1.2 Waypoint generation

We used `cv2.aruco` library for marker detection and tweaked its parameters for correct marker detection. Also we used its corners for marker distance for marker orientation. Then created a waypoint list where all the aruco id and the position of the robot where the aruco is detected is appended and saved in json file as a means of waypoints.

`set_2.py` is the latest file used for aurco marker detection and and waypoint storage.

## Task 1.3 Autonomous Navigation

For autonomous navigation we utilized `move_base` entity within the navigation stack, so the `navigation.launch` files launches the `amcl` and `move_base` nodes, the parameters for `move_base` to generate the local and global cost map as well as to perform path planning and obstacle avoidance and stored within param folder.

This navigation stack in conjugation with `controller.py` script which provides the next goal pose once current is reached within the generated list stored by the `set_2.py` node completes the autonomous navigation task.

## Task 2 Sign Detection

At first we tried to implement this task via openCV but we weren't able to map correct colors and signs. There might be an issue with ambient lighting which wasn’t the case with the simulation and even though some sample pics worked fine but got run time issues while running on the bot
In OpenCv we tried mapping the yellow color and the same base was used for task 3.


## Task 3 Lane Detection and Obstacle avoidance

Lane detection primarily ensured bot tracking by analyzing a single lane's slope, simplifying the process. The camera image was cropped to the lower left side, converted to HSL color space for better color representation, and then transformed into grayscale to emphasize the lane. To mitigate image noise, we applied Gaussian blur and subsequently utilized Canny edge detection. The Hough transforms isolated lane features, providing x and y coordinates for the left lane edges. To represent the left lane as a single line, we averaged the lines and extrapolated the lane trajectory, using the lane slope as a marker to keep the bot on course.

The contour mapping  for wall following was implemented in this task when an obstacle was observed .We took the `/scan` topic from lidar and segmented the lidar scan data into front, left and right by slicing the range list.
Once the obstacle node notifies us about the presence of an obstacle in front of it we perform contour mapping which actually follows the surface of the wall until it finds the newest left lane. Inspired by bug 1 algorithm.

`obstacle.py `: notifies about initial obstacle hit
`lane_test.py` : received slope of the lane detected and computer robot velocities as well as implements contour mapping.
`Image_ingestion_pipeline.py` : Accepts the image and generates the left lane's slope as the output.
