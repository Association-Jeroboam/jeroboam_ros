#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration 
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import ThisLaunchFileDir, PathJoinSubstitution
from launch.actions import IncludeLaunchDescription, GroupAction
from launch_ros.substitutions import FindPackageShare
from launch.conditions import  UnlessCondition
from launch_ros.actions import SetRemap
import rclpy
from rclpy.logging import get_logger
import os


def generate_launch_description():
    rclpy.init()
    logger = get_logger('launch_logger')

    package_name = "jrb_sensors"
    this_pkg = FindPackageShare("jrb_sensors")

    os.environ["ROBOT_NAME"] = "panier"

    robotName = os.environ["ROBOT_NAME"]

    camera_param_path = LaunchConfiguration("camera_param_path")

    declare_camera_param = DeclareLaunchArgument(
        "camera_param_path",
        description="Full path to camera parameter file to load",
        default_value=PathJoinSubstitution(
            ["/home/jeroboam/ros2_ws/src/jeroboam_ros/jrb_sensors/param/", "panier_camera_param.yaml"]
        ),
    )

    cherries_counter =  Node(
        package="jrb_sensors",
        executable="cherries_counter.py",
        output="screen",
        emulate_tty=True
    )

    camera = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        name="v4l2_camera",
        parameters=[camera_param_path],
        output="screen",
        emulate_tty=True
    )

    ld = LaunchDescription()

    ld.add_action(declare_camera_param)

    ld.add_action(cherries_counter)

    ld.add_action(camera)

    return ld
