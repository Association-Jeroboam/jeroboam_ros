#!/usr/bin/env python

"""Launch Webots Jeroboam robot driver."""

import os
import pathlib
import xacro
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch import LaunchDescription
from launch_ros.actions import Node
import launch
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher, Ros2SupervisorLauncher
from webots_ros2_driver.utils import controller_url_prefix
from webots_ros2_driver.urdf_spawner import URDFSpawner, get_webots_driver_node


def get_ros2_nodes(*args):
    package_dir = get_package_share_directory("jrb_webots")
    jrb_description_package_dir = get_package_share_directory("jrb_description")

    robotrouge_xacro_path = os.path.join(
        jrb_description_package_dir, "urdf", "robotrouge.urdf.xacro"
    )
    robotrouge_description = xacro.process_file(
        robotrouge_xacro_path, mappings={"name": "robotrouge"}
    ).toxml()

    spawn_robotrouge = URDFSpawner(
        name="robotrouge",
        robot_description=robotrouge_description,
        relative_path_prefix=os.path.join(jrb_description_package_dir, "urdf"),
        translation="0 0 0.62",
        rotation="0 0 1 -1.5708",
    )

    return [spawn_robotrouge]


def generate_launch_description():
    return LaunchDescription(get_ros2_nodes())
