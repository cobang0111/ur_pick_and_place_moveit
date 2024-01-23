# Universal Robot Pick and Place Example on ROS2 MoveIt 
## ✨Summary
This repository explain the Universal Robot's pick and place example based on `MoveGroupInterface` (cpp) 
<br>
Tested on Ubuntu 22.04 - ROS2 humble - UR3e


## ✨Prerequisite
This repository assumes that you have already connected the Universal Robots (UR) with the Host PC and installed all the official ROS2 packages.
Also, MoveIt2 simple path planning of UR must be possible.
<br>
Please follow the step of below official ROS2 UR Github link.
<br>
https://github.com/LucaBross/simple_moveit2_universal_robots_movement


## ✨Install
Please clone this repository on your UR ROS2 package location
```bash
export COLCON_WS=~/workspace/ros_ur_driver

cd $COLCON_WS

cd src/Universal_Robots_ROS2_Driver 

git clone https://github.com/cobang0111/ur_pick_and_place_moveit.git

cd ../..
```

And then rebuild the packages
```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash
```
 

## ✨Execution
Please open new terminal (2nd terminal) - Calibration and Activating Rviz
<br>
If your UR is not UR3e, then you need to change the `ur_type` in `launch/pick_and_place_moveit_launch.py` and below shell command
```bash
export COLCON_WS=~/workspace/ros_ur_driver

cd $COLCON_WS

source install/setup.bash

ros2 launch ur_calibration calibration_correction.launch.py robot_ip:=<robot_ip> target_filename:="${HOME}/my_robot_calibration.yaml"

ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=<robot_ip> launch_rviz:=false
```
You need to activate the external control program on your PolyScope
<br>
Please open new terminal (3rd terminal) - Activating MoveIt2
```bash
export COLCON_WS=~/workspace/ros_ur_driver

cd $COLCON_WS

source install/setup.bash

ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3e launch_rviz:=true robot_ip:=<robot_ip> reverse_ip:=<host_ip>
```

Input below command on the 1st terminal.
<br>
Please be careful that robot can move unexpect way.
```bash
ros2 launch ur_pick_and_place_moveit pick_and_place_moveit_launch.py
```

## ✨Customizing

You need to modify the code `src/ur_pick_and_place_moveit.cpp` appropriately to your case
<br>
Don't forget to rebuild the package when you modify the code.


## ✨Reference
https://www.youtube.com/watch?v=RaQ8Ibd9vck&t=10s
<br>
https://github.com/LucaBross/simple_moveit2_universal_robots_movement

