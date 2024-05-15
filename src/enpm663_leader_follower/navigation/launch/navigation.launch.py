# from launch import LaunchDescription
# from launch_ros.actions import Node


# def generate_launch_description():
#     ld = LaunchDescription()

#     leader_cmd = Node(
#         package="navigation",
#         executable="leader_navigator",
#         output="screen",
#     )
    
#     follower_cmd = Node(
#         package="navigation",
#         executable="follower_navigator",
#         output="screen",
#     )

#     ld.add_action(leader_cmd)  # Ad a node action to the object
#     ld.add_action(follower_cmd)  # Ad a node action to the object
#     return ld

import os
import rclpy.logging
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    follower_task = LaunchConfiguration("follower", default="init")
    leader_task = LaunchConfiguration("leader", default="init")

    
    # Follower node
    follower_cmd = Node(
        package="navigation",
        executable="follower_navigator",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"follower": follower_task},
        ],
    )

    # Leader node
    leader_cmd = Node(
        package="navigation",
        executable="leader_navigator",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"leader": leader_task},
        ],
    )

    # Nodes to start
    nodes_to_start = [leader_cmd, follower_cmd]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "leader",
            default_value="init",
            description="Task for the leader robot",
            choices=["init", "goal", "waypoints"],
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "follower",
            default_value="init",
            description="Task for the follower robot",
            choices=["init", "goal", "waypoints"],
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
