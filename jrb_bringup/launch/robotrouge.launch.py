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
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import SetRemap
import os


def generate_launch_description():
    package_name = "jrb_bringup"

    this_pkg = FindPackageShare("jrb_bringup")

    camera_param_path = LaunchConfiguration("camera_param_path")
    lidar_param_path = LaunchConfiguration("lidar_param_path")
    can_bridge_param_path = LaunchConfiguration("can_bridge_param_path")
    global_localization = LaunchConfiguration('global_localization')

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory(package_name),
                    "launch",
                    "robot_state_publisher.launch.py",
                )
            ]
        ),
        launch_arguments={"display_meshes": "true"}.items(),
    )

    twist_mux_params = os.path.join(
        get_package_share_directory("jrb_bringup"), "param", "twist_mux.yaml"
    )

    joystick = GroupAction(
        actions=[
            SetRemap(src="/cmd_vel", dst="/cmd_vel_joy"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [ThisLaunchFileDir(), "/joystick.launch.py"]
                ),
            ),
        ]
    )

    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        output="screen",
        parameters=[twist_mux_params],
        remappings=[("cmd_vel_out", "/cmd_vel")],
    )

    marker_publisher_params = os.path.join(
        get_package_share_directory("jrb_bringup"), "param", "marker_publisher.yaml"
    )

    marker_publisher = Node(
        package="jrb_strategy2",
        executable="marker_publisher",
        output="screen",
        parameters=[marker_publisher_params],
        remappings=[("markers", "/debug/table_mesh")],
    )

    static_transform_broadcaster = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        condition=UnlessCondition(global_localization)
    )

    return LaunchDescription(
        [
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
                "global_localization",
                description="Use a global localization node such as amcl to have the tf map->odom",
                default_value="False"
            ),
            rsp,
            static_transform_broadcaster,
            Node(
                package="v4l2_camera",
                executable="v4l2_camera_node",
                name="v4l2_camera",
                parameters=[camera_param_path],
                output="screen",
            ),
            twist_mux,
            joystick,
            marker_publisher,
            Node(
                package="jrb_sensors",
                executable="sample_detector",
                output="screen",
            ),
            # Node(
            #     package="jrb_actuators",
            #     executable="actuators",
            #     output="screen",
            # ),
            Node(
                package="jrb_actuators",
                executable="teleop_actuators_joy",
                output="screen",
            ),
            # Node(
            #     package="jrb_localization",
            #     executable="map_manager",
            #     output="screen",
            # ),
            Node(
                package="jrb_actuators",
                executable="teleop_actuators_joy",
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
                package="jrb_can_bridge",
                executable="jrb_can_bridge",
                output="screen",
                parameters=[can_bridge_param_path],
            ),
            Node(
                package="rplidar_ros2",
                executable="rplidar_scan_publisher",
                parameters=[lidar_param_path],
                output="screen",
            ),
            Node(
                package="jrb_sensors",
                executable="obstacle_detector.py",
                output="screen",
            ),
            Node(
                package="jrb_hardware_bridge",
                executable="gpio_node",
                output="screen",
            ),
            # Node(package="jrb_strategy", executable="eurobot", output="screen"),
        ],
    )
