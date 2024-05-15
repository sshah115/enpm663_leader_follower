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

    # Set the path to this package.
    final_project_pkg = FindPackageShare(package="final_project").find("final_project")
    final_project_launch_dir = os.path.join(final_project_pkg, "launch")

    # Set the path to the sensor files
    user_config_path = os.path.join(final_project_pkg, "config", "sensors.yaml")

    if not os.path.exists(user_config_path):
        rclpy.logging.get_logger("Launch File").fatal(
            f"Sensor configuration 'sensors.yaml' not found in config directory: {user_config_path}."
        )
        exit()

    # Sensor TF
    sensor_broadcaster_cmd = Node(
        package="final_project",
        executable="sensor_tf_broadcaster.py",
        output="screen",
        arguments=[user_config_path],
        parameters=[
            {"use_sim_time": use_sim_time},
        ],
    )

    # Environment Startup
    environment_startup_cmd = Node(
        package="final_project",
        executable="environment_startup_node.py",
        output="screen",
        parameters=[
            {"user_config_path": user_config_path},
            {"use_sim_time": use_sim_time},
        ],
    )

    # TF relay
    tf_relay_startup_cmd = Node(
        package="final_project",
        executable="tf_relay_main.py",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
        ],
    )

    # Part spawner
    part_spawner_cmd = Node(
        package="final_project", executable="part_spawner.py", output="screen"
    )

    # Aruco detection
    aruco_detection_cmd = Node(
        package="ros2_aruco", executable="aruco_node", output="screen"
    )

    leader_transform_cmd = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=[
            "0",  # x
            "0",  # y
            "0",  # z
            "0",  # r
            "0",  # p
            "0",  # y
            "world",  # frame_id
            "leader/map",  # child_frame_id
        ],
    )

    leader_odom_map_cmd = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=[
            "0",  # x
            "0",  # y
            "0",  # z
            "0",  # r
            "0",  # p
            "0",  # y
            "leader/map",  # frame_id
            "leader/odom",  # child_frame_id
        ],
    )

    follower_transform_cmd = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=[
            "0",  # x
            "0",  # y
            "0",  # z
            "0",  # r
            "0",  # p
            "0",  # y
            "world",  # frame_id
            "follower/map",  # child_frame_id
        ],
    )

    follower_odom_map_cmd = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=[
            "0",  # x
            "0",  # y
            "0",  # z
            "0",  # r
            "0",  # p
            "0",  # y
            "follower/map",  # frame_id
            "follower/odom",  # child_frame_id
        ],
    )

    # Start multi robots
    multi_robots_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(final_project_launch_dir, "multi_robots.launch.py")
        ),
    )

    # Nodes to start
    nodes_to_start = [
        multi_robots_cmd,
        environment_startup_cmd,
        part_spawner_cmd,
        sensor_broadcaster_cmd,
        # static_transform_cmd,
        aruco_detection_cmd,
        leader_transform_cmd,
        leader_odom_map_cmd,
        follower_odom_map_cmd,
        follower_transform_cmd,
        tf_relay_startup_cmd,
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "sensor_config",
            default_value="sensors",
            description="name of user configuration file",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz",
            default_value="false",
            description="Whether to start RViz",
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
