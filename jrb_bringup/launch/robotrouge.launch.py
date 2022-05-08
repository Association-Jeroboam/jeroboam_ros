#!/usr/bin/env python3

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import ThisLaunchFileDir
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    jrb_bringup_pkg_share = FindPackageShare("jrb_bringup").find("jrb_bringup")
    camera_param_path = LaunchConfiguration(
        'camera_param_path',
        default=os.path.join(jrb_bringup_pkg_share, 'param', 'robotrouge_camera_param.yaml'))

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value=use_sim_time,
                description="Use simulation (Gazebo) clock if true",
            ),
            DeclareLaunchArgument(
                'camera_param_path',
                default_value=camera_param_path,
                description='Full path to camera parameter file to load'
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [ThisLaunchFileDir(), "/robot_state_publisher.launch.py"]
                ),
                launch_arguments={"use_sim_time": use_sim_time}.items(),
            ),
            Node(
                package="usb_cam",
                executable="usb_cam_node_exe",
                name="camera",
                parameters=[camera_param_path],
                output="screen",
            ),
            Node(
                package="jrb_sample_detector",
                executable="sample_detector",
                output="screen",
            ),
            Node(
                package="jrb_actuators",
                executable="actuators",
                output="screen",
            ),
            # Node(
            #     package="rviz2",
            #     executable="rviz2",
            #     name="rviz2",
            #     # arguments=['-d', rviz_config_dir],
            #     output="screen",
            # ),
        ],
    )
