

# Prerequisites for the Final Project

## Software

- Ubuntu 20.04
- ROS Galactic

## Install Dependencies

```bash
sudo apt install ros-galactic-nav2-'*' ros-galactic-navigation2 ros-galactic-turtlebot3'*'
```


## Create a new workspace and clone the repository

It is recommended to create a new workspace for the final project. The following commands will create a new workspace, clone the repository, and build the workspace.

```bash
mkdir -p ~/enpm605_final_ws/src
cd ~/enpm605_final_ws/src
git clone -b final_project https://github.com/zeidk/enpm605_spring2024_ros.git
cd ~/enpm605_final_ws
colcon build
source install/setup.bash
```
## Edit the .bashrc file

* Edit the .bashrc file to source the new workspace
* Add the following line:
```bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:<path to the models folder>
# Example:
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/zeid/enpm605_final_ws/src/enpm605_spring2024_ros/enpm605_final_project/models
```


## OpenCV Installation

OpenCV is required for the final project. To install OpenCV, run the following command:

```bash
sudo apt install python3-pip -y
pip3 uninstall opencv-python
pip3 uninstall opencv-contrib-python
pip3 install opencv-contrib-python
```

## Test the installation

```bash
ros2 launch final_project final_project.launch.py
```
The Gazebo environment should load with two robots (two RViz windows should also load). If you do not see the robots then there is a good chance that the command ```export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:<path to the models folder>``` was not added to the .bashrc file or the path to the `models` folder is incorrect.

![Simulation Environment](figures/gazebo.jpg)

## Check OpenCV installation

After starting the simulation, run the following command to check if OpenCV is installed correctly:
```bash
ros2 topic echo aruco_poses
```

You should see the ArUco poses being published to the `aruco_poses` topic (see output examples below).

```terminal
--
header:
  stamp:
    sec: 115
    nanosec: 171000000
  frame_id: follower/camera_rgb_optical_frame
poses:
- position:
    x: -0.135574710408108
    y: -0.3644814647163148
    z: 2.11904374003013
  orientation:
    x: 0.9999708203872018
    y: 0.00047636197352801814
    z: -0.0029222579923607874
    w: -0.00704214893644064
---
```

### Troubleshooting


As a sanity check, ensure there are no [ERROR] messages in the `~/.ros/log/[most_recent_folder]/launch.log` file. If there are, report them to the instructor.

## Test Navigation

To test the navigation, run the following commands:

```bash
ros2 launch final_project final_project.launch.py
ros2 launch navigation navigation.launch.py follower:=goal leader:=goal
```

You should see both robots going to a goal location. 