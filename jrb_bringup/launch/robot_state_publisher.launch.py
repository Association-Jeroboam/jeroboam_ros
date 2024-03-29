#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    Command,
)
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    this_pkg = FindPackageShare("jrb_bringup")

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_gui = LaunchConfiguration("use_gui")
    display_meshes = LaunchConfiguration("display_meshes")
    urdf_file = LaunchConfiguration(
        "urdf_file",
        default=PathJoinSubstitution(
            [FindPackageShare("jrb_description"), "urdf", "robotrouge.urdf.xacro"]
        ),
    )
    joint_state_publisher_param_file = LaunchConfiguration(
        "robot_state_publisher_param_file",
        default=PathJoinSubstitution(
            [this_pkg, "param", "joint_state_publisher_param.yaml"]
        ),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                description="Use simulation clock if true",
                default_value="False",
            ),
            DeclareLaunchArgument(
                "urdf_file",
                description="urdf file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("jrb_description"),
                        "urdf",
                        "robotrouge.urdf.xacro",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "use_gui",
                description="Launch joint_state_publisher_gui instead of joint_state_publisher",
                default_value="False",
            ),
            DeclareLaunchArgument(
                "display_meshes",
                description="Display detailed meshes in urdf",
                default_value="True",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "robot_description": ParameterValue(
                            Command(
                                [
                                    "xacro ",
                                    urdf_file,
                                    " display_meshes:=",
                                    display_meshes,
                                ]
                            ),
                            value_type=str,
                        ),
                        "use_sim_time": use_sim_time,
                    }
                ],
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                output="screen",
                condition=UnlessCondition(use_gui),
                parameters=[
                    joint_state_publisher_param_file,
                    {
                        "use_sim_time": use_sim_time,
                    },
                ],
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
                output="screen",
                condition=IfCondition(use_gui),
                parameters=[
                    joint_state_publisher_param_file,
                    {
                        "use_sim_time": use_sim_time,
                    },
                ],
            ),
        ]
    )
