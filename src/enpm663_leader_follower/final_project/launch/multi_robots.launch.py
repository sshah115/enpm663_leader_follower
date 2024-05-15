# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Example for spawing multiple robots in Gazebo.

This is an example on how to create a launch file for spawning multiple robots into Gazebo
and launch multiple instances of the navigation stack, each controlling one robot.
The robots co-exist on a shared environment and are controlled by independent nav stacks
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution


def generate_launch_description():
    # Get the launch directory
    final_project_pkg = get_package_share_directory("final_project")
    final_project_launch_dir = os.path.join(final_project_pkg, "launch")

    # Names and poses of the robots
    robots = [
        {"name": "leader", "x_pose": 2.0, "y_pose": 1.0, "z_pose": 0.01},
        {"name": "follower", "x_pose": 1.0, "y_pose": 1.0, "z_pose": 0.01},
    ]

    # Simulation settings
    world = LaunchConfiguration("world")
    simulator = LaunchConfiguration("simulator")

    # In this example all robots are launched with the same settings
    map_yaml_file = LaunchConfiguration("map")
    autostart = LaunchConfiguration("autostart")
    rviz_config_file = LaunchConfiguration("rviz_config")
    use_robot_state_pub = LaunchConfiguration("use_robot_state_pub")
    use_rviz = LaunchConfiguration("use_rviz")
    log_settings = LaunchConfiguration("log_settings", default="true")

    # Declare the launch arguments
    declare_world_cmd = DeclareLaunchArgument(
        "world",
        default_value=os.path.join(final_project_pkg, "world", "final.world"),
        description="Full path to world file to load",
    )

    declare_simulator_cmd = DeclareLaunchArgument(
        "simulator",
        default_value="gazebo",
        description="The simulator to use (gazebo or gzserver)",
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(final_project_pkg, "map", "final_map.yaml"),
        description="Full path to map file to load",
    )

    declare_leader_params_file_cmd = DeclareLaunchArgument(
        "leader_params_file",
        default_value=os.path.join(final_project_pkg, "params", "leader_params.yaml"),
        description="Full path to the ROS2 parameters file to use for leader launched nodes",
    )

    declare_follower_params_file_cmd = DeclareLaunchArgument(
        "follower_params_file",
        default_value=os.path.join(final_project_pkg, "params", "follower_params.yaml"),
        description="Full path to the ROS2 parameters file to use for follower launched nodes",
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the stacks",
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        "rviz_config",
        default_value=os.path.join(
            final_project_pkg, "rviz", "nav2_namespaced_view.rviz"
        ),
        description="Full path to the RVIZ config file to use.",
    )

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        "use_robot_state_pub",
        default_value="True",
        description="Whether to start the robot state publisher",
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz", default_value="True", description="Whether to start RVIZ"
    )

    # Start Gazebo with plugin providing the robot spawing service
    start_gazebo_cmd = ExecuteProcess(
        cmd=[
            simulator,
            "--verbose",
            "-s",
            "libgazebo_ros_init.so",
            "-s",
            "libgazebo_ros_factory.so",
            world,
        ],
        output="screen",
    )

    # Define commands for spawing the robots into Gazebo
    spawn_robots_cmds = []
    robot_name = "waffle"
    for robot in robots:
        if robot["name"] == "leader":
            robot_name = "waffle_leader"
        elif robot["name"] == "follower":
            robot_name = "waffle_follower"
        
        spawn_robots_cmds.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(final_project_launch_dir, "spawn_tb3_launch.py")
                ),
                launch_arguments={
                    "x_pose": TextSubstitution(text=str(robot["x_pose"])),
                    "y_pose": TextSubstitution(text=str(robot["y_pose"])),
                    "z_pose": TextSubstitution(text=str(robot["z_pose"])),
                    "robot_name": robot["name"],
                    "turtlebot_type": robot_name,
                    # "turtlebot_type": turtlebot_type,
                }.items(),
            )
        )

    # Define commands for launching the navigation instances
    nav_instances_cmds = []
    for robot in robots:
        params_file = LaunchConfiguration(f"{robot['name']}_params_file")

        group = GroupAction(
            [
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(final_project_launch_dir, "rviz_launch.py")
                    ),
                    condition=IfCondition(use_rviz),
                    launch_arguments={
                        "namespace": TextSubstitution(text=robot["name"]),
                        "use_namespace": "True",
                        "rviz_config": rviz_config_file,
                    }.items(),
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(
                            final_project_launch_dir,
                            "tb3_simulation_launch.py",
                        )
                    ),
                    launch_arguments={
                        "namespace": robot["name"],
                        "use_namespace": "True",
                        "map": map_yaml_file,
                        "use_sim_time": "True",
                        "params_file": params_file,
                        "autostart": autostart,
                        "use_rviz": "False",
                        "use_simulator": "False",
                        "headless": "False",
                        "use_robot_state_pub": "True",
                    }.items(),
                ),
                LogInfo(
                    condition=IfCondition(log_settings),
                    msg=["Launching ", robot["name"]],
                ),
                LogInfo(
                    condition=IfCondition(log_settings),
                    msg=[robot["name"], " map yaml: ", map_yaml_file],
                ),
                LogInfo(
                    condition=IfCondition(log_settings),
                    msg=[robot["name"], " params yaml: ", params_file],
                ),
                LogInfo(
                    condition=IfCondition(log_settings),
                    msg=[robot["name"], " rviz config file: ", rviz_config_file],
                ),
                LogInfo(
                    condition=IfCondition(log_settings),
                    msg=[
                        robot["name"],
                        " using robot state pub: ",
                        use_robot_state_pub,
                    ],
                ),
                LogInfo(
                    condition=IfCondition(log_settings),
                    msg=[robot["name"], " autostart: ", autostart],
                ),
            ]
        )

        nav_instances_cmds.append(group)

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_leader_params_file_cmd)
    ld.add_action(declare_follower_params_file_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)

    # Add the actions to start gazebo, robots and simulations
    ld.add_action(start_gazebo_cmd)

    for spawn_robot_cmd in spawn_robots_cmds:
        ld.add_action(spawn_robot_cmd)

    for simulation_instance_cmd in nav_instances_cmds:
        ld.add_action(simulation_instance_cmd)

    return ld