```bash
rosrun bunker_bringup bringup_can2usb.bash
roslaunch bunker_bringup bunker_control_frontier.launch 
roslaunch gromi_scan_planning bunker_run.launch 
# pub_tf arguments in bunker_control_base.launch was declared 'string' not 'bool'
# LeGO-LoAM simulation parameter to true
```


```bash
roslaunch bunker_bringup bunker_robot_base.launch

# launch bunker robot control -> generate /smoother_cmd_vel -> /cmd_vel
roslaunch bunker_bringup bunker_control.launch

roslaunch bunker_bringup bunker_teleop_keyboard.launch

roslaunch gromi_scan_planning bunker_run.launch



rosrun bunker_bringup bringup_can2usb.bash

roslaunch bunker_bringup bunker_control_traversability.launch

roslaunch traversability_mapping online.launch
```
