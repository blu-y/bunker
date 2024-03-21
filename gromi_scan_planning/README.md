# GROMI SCAN PLANNING
This is based on the jackal velodyne simulation package.

## Introduction
####  Objective
The goal is to autonomously navigate and map an unknown area using SLAM and a frontier exploration algorithm based on laser data from a Velodyne VLP-16 LIDAR.

#### How it works
It is demostrated on the simulated environment.
First, it tries to find the frontier locations which is the border of the free space and unknown space.
It then decides the next goal location among the frontier locations and calculates the best scan location near the next goal location.
For your information, in the videos, the blue marks are the frontier locations and the red circles are the designated scan locations.

#### Installation
- Tested on Ubuntu 18.04 with ROS Melodic
- To run this code, please install the following packages:
```bash
sudo apt-get install ros-melodic-navigation
sudo apt-get install ros-melodic-tf2-sensor-msgs
sudo apt-get install ros-melodic-jackal-*
sudo apt-get install ros-melodic-velodyne-*
sudo apt-get install ros-melodic-octomap-mapping 
sudo apt-get install ros-melodic-pointcloud-to-laserscan
sudo apt-get install python-scipy

python -m pip install --upgrade pip
pip install scikit-build
pip install cmake
pip install opencv-python
```

## How to run
- simulation with gmapping : `roslaunch gromi_scan_planning simulation_gromi_gmaaping.launch`
- simulation with lego-loam : `roslaunch gromi_scan_planning simulation_gromi.launch`
- real gromi hardware : `roslaunch gromi_scan_planning gromi_scanning.launch`

#### Nodes
##### Exploration Node
[`exlore.py`](src/explore.py)

This node is the brain that computes what the next goal position and scan location should be the robot should go to.

Parameters 
- `self.minScanDist = 5` : no duplicate scan within 5 m
- `self.min_bndX = -10` : Target area X min bound
- `self.max_bndX = 10` : Target area X max bound
- `self.min_bndY = -10` : Target area Y min bound
- `self.max_bndY = 10` : Target area Y max bound
    
Subscribed Topics:
- `/map` for updating the current map
- `/move_base/global_costmap/costmap` for updating the current global costmap
- `/move_base/global_costmap/costmap_updates` for updating the current global costmap
- `/move_base/result` for determining when the robot has reached its goal position
- `/move_base/feedback` for determining where the robot is in the map at any given point in time
- `/scan_cloud` for subscribing the front 2d Lidar point cloud data
- `/octomap_point_cloud_centers` for subscribing octomap point cloud center data 
- `/start_scan` for subscribing to start the static scanning process  

Published Topic: 
- `/move_base_simple/goal` for the next goal location
- `/visualization_marker` for visualizing the frontier locations (blue markers)
- `/scan_locations` for visualizing the scan locations (red circles)
- `/finish_scan` for stopping the static scanning process 


[`gromi_static_scan.py`](src/gromi_static_scan.py)

This node is to take pictures and run/stop the rotation scanning motor.

Parameters 
- `self.rotation_time = 100` : scanning time for one rotation
- `self.num_of_pic = 8` : number of pictures that take for one rotation
- `self.time_between_pics` : time between each picture
- `self.trigger` : trigger to take pictures and run/stop the rotation scanning motor
- `self.cnt` : number of pictures that take at current
    
Subscribed Topics:
- `/start_scan` for receiving command that start the static scanning process

Published Topic: 
- `/run_scan_motor` for sending command that run/stop the rotation scanning motor
- `/finish_scan` for sending the command that finish the static scanning process


[`motor_controller.py`](src/motor_controller.py)

This node is to make differential wheel speed.

Parameters 
- `self.mps_2_data_coeff = 212.6` : Conversion factor for converting m/s to a data value for the motor controller. Setting this to a higher value will lead to the robot moving faster for the same speed input.
- `self.radps_2_data_coeff = 233.9` : Conversion factor for converting rad/s to a data value offset for the motor controller. Setting this to a higher value will lead to the robot rotating faster for the same angular speed input. 
- `self.smoothing_coeff = 0.8` : The smoothing coefficient controls how much smoothing is applied. Must be between [0, 1). Setting to 0.0 would apply zero smoothing while setting to 1.0 would make the system unresponsive. Keep in mind that higher publishing frequency requires a higher smoothing coefficient for the same result
- `self.pub_frequency = 20.0` : The controller's running frequency in Hz
    
Subscribed Topics:
- `/cmd_vel/nav` for receiving velocity command 

Published Topic: 
- `/left_wheel_vel` for sending the left wheel velocity command
- `/right_wheel_vel` for sending the right wheel velocity command


[`scan_motor_controller.py`](src/scan_motor_controller.py)

This node is to make rotation of scanning system.

Parameters 
- `self.start_command = "0106007D40002812"` : define the start commands for the scanning motor
- `self.stop_command = "0106007D0020180A"` : define the stop commands for the scanning motor
- `self.run_motor` : true - run the scanning motor, false - stop the scanning motor
- `self.pub_frequency = 20.0` : The controller's running frequency in Hz
    
Subscribed Topics:
- `/run_scan_motor` for receiving command that start to run the scanning motor 


## Demo Video
- A video of Jackal exploring a simulated environment (rectangular objects) in Gazebo and visualizing in RViZ. [here](https://www.youtube.com/watch?v=RbIVOsBVjnk).
- A video of Jackal exploring a simulated environment (cylinder objects) in Gazebo and visualizing in RViZ. [here](https://www.youtube.com/watch?v=74FLvJFuMgo).

