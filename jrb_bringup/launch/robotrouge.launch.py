#!/usr/bin/env python3

is_raspi_ = "True"
try:
    import RPi.GPIO as GPIO
except RuntimeError:
    is_raspi_ = "False"

from operator import is_
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import ThisLaunchFileDir, PathJoinSubstitution
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
import os


def generate_launch_description():
    this_pkg = FindPackageShare("jrb_bringup")

    use_sim_time = LaunchConfiguration("use_sim_time")
    is_raspi = LaunchConfiguration("is_raspi", default=is_raspi_)
    camera_param_path = LaunchConfiguration("camera_param_path")
    lidar_param_path = LaunchConfiguration("lidar_param_path")

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
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [ThisLaunchFileDir(), "/robot_state_publisher.launch.py"]
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "use_gui": "False",
                }.items(),
            ),
            Node(
                package="usb_cam",
                executable="usb_cam_node_exe",
                name="camera",
                parameters=[camera_param_path],
                output="screen",
            ),
            Node(
                package="jrb_sensors",
                executable="sample_detector",
                output="screen",
            ),
            Node(
                package="jrb_actuators",
                executable="actuators",
                output="screen",
            ),
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
                executable="raspi_gpio",
                output="screen",
                condition=IfCondition(is_raspi),
            ),
        ],
    )