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
from launch.conditions import UnlessCondition
from launch_ros.actions import SetRemap
import rclpy
from rclpy.logging import get_logger
import os


def generate_launch_description():
    rclpy.init()
    logger = get_logger("launch_logger")

    package_name = "jrb_bringup"
    this_pkg = FindPackageShare("jrb_bringup")

    if not os.environ.get("ROBOT_NAME"):
        logger.warn("ROBOT_NAME is not set, default to robotrouge")
        os.environ["ROBOT_NAME"] = "robotrouge"

    robotName = os.environ["ROBOT_NAME"]
    isRobotrouge = robotName == "robotrouge"

    camera_param_path = LaunchConfiguration("camera_param_path")
    lidar_param_path = LaunchConfiguration("lidar_param_path")
    can_bridge_param_path = LaunchConfiguration("can_bridge_param_path")
    global_localization = LaunchConfiguration("global_localization")

    declare_camera_param = DeclareLaunchArgument(
        "camera_param_path",
        description="Full path to camera parameter file to load",
        default_value=PathJoinSubstitution(
            [this_pkg, "param", "robotrouge_camera_param.yaml"]
        ),
    )

    declare_lidar_param = DeclareLaunchArgument(
        "lidar_param_path",
        description="Full path to camera parameter file to load",
        default_value=PathJoinSubstitution([this_pkg, "param", "lidar_param.yaml"]),
    )

    declare_can_bridge_param = DeclareLaunchArgument(
        "can_bridge_param_path",
        description="Full path to can_bridge parameter file to load",
        default_value=PathJoinSubstitution(
            [this_pkg, "param", robotName + "_can_bridge_param.yaml"]
        ),
    )
    declare_global_localization = DeclareLaunchArgument(
        "global_localization",
        description="Use a global localization node such as amcl to have the tf map->odom",
        default_value="False",
    )

    twist_mux_params = os.path.join(
        get_package_share_directory("jrb_bringup"), "param", "twist_mux.yaml"
    )

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
        launch_arguments={
            "display_meshes": "true",
            "urdf_file": PathJoinSubstitution(
                [FindPackageShare("jrb_description"), "urdf", robotName + ".urdf.xacro"]
            ),
        }.items(),
    )

    can_bridge = Node(
        package="jrb_can_bridge",
        executable="jrb_can_bridge",
        output="screen",
        parameters=[
            can_bridge_param_path,
            {"robot_name": os.environ.get("ROBOT_NAME")},
        ],
        emulate_tty=True,
    )

    go_to_goal = Node(
        package="jrb_control",
        executable="go_to_goal",
        output="screen",
        emulate_tty=True,
    )

    lidar = Node(
        package="rplidar_ros2",
        executable="rplidar_scan_publisher",
        parameters=[lidar_param_path],
        output="screen",
        emulate_tty=True,
    )

    obstacle_detector = Node(
        package="jrb_sensors",
        executable="obstacle_detector.py",
        output="screen",
        emulate_tty=True,
    )

    gpio_node = Node(
        package="jrb_hardware_bridge",
        executable="gpio_node",
        output="screen",
        parameters=[{"robot_name": os.environ.get("ROBOT_NAME")}],
        emulate_tty=True,
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
        emulate_tty=True,
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
        emulate_tty=True,
    )

    static_transform_broadcaster = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="screen",
        arguments=[
            "--x",
            "0.0",
            "--y",
            "0.0",
            "--z",
            "0.0",
            "--roll",
            "0.0",
            "--pitch",
            "0.0",
            "--yaw",
            "0.0",
            "--frame-id",
            "map",
            "--child-frame-id",
            "odom",
        ],
        condition=UnlessCondition(global_localization),
        emulate_tty=True,
    )

    screen_manager = Node(
        package="jrb_screen",
        executable="screen_manager",
        output="screen",
        emulate_tty=True,
    )

    camera = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        name="v4l2_camera",
        parameters=[camera_param_path],
        output="screen",
        emulate_tty=True,
    )

    sample_detector = Node(
        package="jrb_sensors",
        executable="sample_detector",
        output="screen",
        emulate_tty=True,
    )

    actuators = Node(
        package="jrb_actuators",
        executable="actuators.py",
        output="screen",
        parameters=[{"robot_name": os.environ.get("ROBOT_NAME")}],
        emulate_tty=True,
    )

    teleop_actuators_joy = Node(
        package="jrb_actuators",
        executable="teleop_actuators_joy",
        output="screen",
        parameters=[{"robot_name": os.environ.get("ROBOT_NAME")}],
        emulate_tty=True,
    )

    strategy_robotbleu = Node(
        package="jrb_strategy",
        executable="robotbleu",
        output="screen",
        emulate_tty=True,
    )

    strategy_robotrouge = Node(
        package="jrb_strategy",
        executable="robotrouge",
        output="screen",
        emulate_tty=True,
    )

    ld = LaunchDescription()

    ld.add_action(declare_camera_param)
    ld.add_action(declare_can_bridge_param)
    ld.add_action(declare_lidar_param)
    ld.add_action(declare_global_localization)

    ld.add_action(rsp)
    ld.add_action(static_transform_broadcaster)
    ld.add_action(can_bridge)
    ld.add_action(gpio_node)
    ld.add_action(lidar)
    ld.add_action(obstacle_detector)
    ld.add_action(twist_mux)
    ld.add_action(joystick)
    ld.add_action(marker_publisher)
    ld.add_action(screen_manager)
    ld.add_action(actuators)
    ld.add_action(teleop_actuators_joy)
    # ld.add_action(go_to_goal)

    if isRobotrouge:
        ld.add_action(camera)
        ld.add_action(sample_detector)
        ld.add_action(strategy_robotrouge)
    # else:
    #     ld.add_action(strategy_robotbleu)

    return ld
