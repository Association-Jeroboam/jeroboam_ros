#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    jrb_description_pkg_share = FindPackageShare("jrb_description").find("jrb_description")
    urdf_dir = os.path.join(jrb_description_pkg_share, "urdf")
    urdf_file = os.path.join(urdf_dir, "robotrouge.urdf")

    jrb_bringup_pkg_share = FindPackageShare("jrb_bringup").find("jrb_bringup")
    param_dir = os.path.join(jrb_bringup_pkg_share, "param")
    robot_state_publisher_param_file = os.path.join(param_dir, "robotrouge_joint_state_publisher_param.yaml")
    print(robot_state_publisher_param_file)

    with open(urdf_file, "r") as infp:
        robot_desc = infp.read()

    rsp_params = {"robot_description": robot_desc}

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock if true",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[rsp_params, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                output="screen",
                arguments=[urdf_file],
                parameters=[robot_state_publisher_param_file]
            ),
            # Node(
            #     package="joint_state_publisher_gui",
            #     executable="joint_state_publisher_gui",
            #     output="screen",
            #     arguments=[urdf_file],
            # ),
        ]
    )
