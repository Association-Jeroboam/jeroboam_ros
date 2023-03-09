#!/usr/bin/env python3

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import ThisLaunchFileDir, PathJoinSubstitution
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
import os


def generate_launch_description():
    this_pkg = FindPackageShare("jrb_bringup")

    use_sim_time = LaunchConfiguration("use_sim_time")
    camera_param_path = LaunchConfiguration("camera_param_path")
    lidar_param_path = LaunchConfiguration("lidar_param_path")
    can_bridge_param_path = LaunchConfiguration("can_bridge_param_path")
    sim_motionboard = LaunchConfiguration("sim_motionboard")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                description="Use simulation (Gazebo) clock if true",
                default_value="False",
            ),
            DeclareLaunchArgument(
                "camera_param_path",
                description="Full path to camera parameter file to load",
                default_value=PathJoinSubstitution(
                    [this_pkg, "param", "robotrouge_camera_param.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "lidar_param_path",
                description="Full path to camera parameter file to load",
                default_value=PathJoinSubstitution(
                    [this_pkg, "param", "lidar_param.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "can_bridge_param_path",
                description="Full path to can_bridge parameter file to load",
                default_value=PathJoinSubstitution(
                    [this_pkg, "param", "robotrouge_can_bridge_param.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "sim_motionboard",
                description="Simulate motionboard with a perfect Twist command to Odometry state",
                default_value="False",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [ThisLaunchFileDir(), "/robot_state_publisher.launch.py"]
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "use_gui": "False",
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [ThisLaunchFileDir(), "/joystick.launch.py"]
                )
            ),
            # Node(
            #     package="usb_cam",
            #     executable="usb_cam_node_exe",
            #     name="camera",
            #     parameters=[camera_param_path],
            #     output="screen",
            # ),
            # Node(
            #     package="jrb_sensors",
            #     executable="sample_detector",
            #     output="screen",
            # ),
            # Node(
            #     package="jrb_actuators",
            #     executable="actuators",
            #     output="screen",
            # ),
            Node(
                package="jrb_localization",
                executable="map_manager",
                output="screen",
            ),
            Node(
                package="jrb_screen",
                executable="screen_manager",
                output="screen",
            ),
            Node(
                package="jrb_control",
                executable="go_to_goal",
                output="screen",
            ),
            Node(
                package="jrb_control",
                executable="simulated_motionboard",
                output="screen",
                condition=IfCondition(sim_motionboard),
            ),
            Node(
                package="jrb_can_bridge",
                executable="jrb_can_bridge",
                output="screen",
                parameters=[can_bridge_param_path],
                condition=UnlessCondition(sim_motionboard),
            ),
            Node(
                package="rplidar_ros2",
                executable="rplidar_scan_publisher",
                parameters=[lidar_param_path],
                output="screen",
            ),
            Node(
                package="jrb_sensors",
                executable="obstacle_detector",
                output="screen",
            ),
            Node(
                package="jrb_hardware_bridge",
                executable="gpio_node",
                output="screen",
            ),
            Node(package="jrb_strategy", executable="eurobot", output="screen"),
        ],
    )
