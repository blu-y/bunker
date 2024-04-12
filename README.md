
### Real world
```
rosrun bunker_bringup bringup_can2usb.bash
roslaunch gromi_scan_planning runall.launch
```

### Simulation (/velodyne_points)
```
rosbag play <bagfilename>.bag --clock --topic /velodyne_points
roslaunch gromi_scan_planning runall.launch sim:=true # for simulation with /velodyne_points
```

### Playing bag file
```
rosbag play <bagfilename>.bag --clock
rviz -d ./src/gromi_scan_planning/rviz/test.rviz
```
