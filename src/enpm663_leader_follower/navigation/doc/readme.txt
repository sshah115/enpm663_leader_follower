Instruction:

Required package:
PyKDL

Installation command:
sudo apt-get install python3-pykdl


Remove build, log, install:
cd ~/enpm605_final_ws
rm -r build/ log/ install/

Install dependencies: rosdep install --from-paths src -y --ignore-src
Build packages: colcon build
Source setups: source install/setup.bash

Commands to run the nodes:
Terminal window1: ros2 launch final_project final_project.launch.py
Terminal window2: ros2 launch navigation navigation.launch.py follower:=init leader:=waypoints

Notes:
The robots detects each other as obstacle once they get closer to each other and automatically reroutes, specially in the initial phase. 
Couldn't provide solution for that.

Server were removed from the leader_navigator as it is redundant as per latest implementation.