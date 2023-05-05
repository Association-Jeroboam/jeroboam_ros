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
        jrb_description_package_dir, "urdf", "myrobot.urdf"
    )
    robotrouge_description = xacro.process_file(
        robotrouge_xacro_path, mappings={"name": "robotrouge"}
    ).toxml()

    ros2_control_params = os.path.join(package_dir, "resource", "ros2control.yml")
    use_sim_time = LaunchConfiguration("use_sim_time", default=True)

    spawn_robotrouge = URDFSpawner(
        name="robotrouge",
        robot_description=robotrouge_description.replace(
            "package://jrb_description/", "package://"
        ),
        relative_path_prefix=os.path.join(jrb_description_package_dir, "urdf"),
        translation="6 0 0",
        rotation="0 0 1 -1.5708",
    )

    # TODO: Revert once the https://github.com/ros-controls/ros2_control/pull/444 PR gets into the release
    controller_manager_timeout = ["--controller-manager-timeout", "50"]

    diffdrive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["diffdrive_controller"] + controller_manager_timeout,
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["joint_state_broadcaster"] + controller_manager_timeout,
    )

    mappings = [("/diffdrive_controller/cmd_vel_unstamped", "/cmd_vel")]
    mappings.append(("/diffdrive_controller/odom", "/odom"))

    robotrouge_driver = Node(
        package="webots_ros2_driver",
        executable="driver",
        output="screen",
        additional_env={
            "WEBOTS_CONTROLLER_URL": controller_url_prefix() + "robotrouge"
        },
        parameters=[
            {
                "robot_description": robotrouge_description,
                "use_sim_time": use_sim_time,
                "set_robot_state_publisher": True,
            },
            ros2_control_params,
        ],
        remappings=mappings,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": '<robot name=""><link name=""/></robot>'}],
        # TODO investigate this:
        # parameters=[{"robot_description": robotrouge_description}],
    )

    # footprint_publisher = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     output="screen",
    #     arguments=["0", "0", "0", "0", "0", "0", "base_link", "base_footprint"],
    # )

    return [
        spawn_robotrouge,
        joint_state_broadcaster_spawner,
        diffdrive_controller_spawner,
        robot_state_publisher,
        # footprint_publisher,
        # Launch the driver node once the URDF robot is spawned.
        # You might include other nodes to start them with the driver node.
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessIO(
                target_action=spawn_robotrouge,
                on_stdout=lambda event: get_webots_driver_node(
                    event, [robotrouge_driver]
                ),
            )
        ),
    ]


def generate_launch_description():
    package_dir = get_package_share_directory("jrb_webots")
    world = LaunchConfiguration("world")

    webots = WebotsLauncher(world=PathJoinSubstitution([package_dir, "worlds", world]))

    ros2_supervisor = Ros2SupervisorLauncher()

    # The following line is important!
    # This event handler respawns the ROS 2 nodes on simulation reset (supervisor process ends).
    reset_handler = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=ros2_supervisor,
            on_exit=get_ros2_nodes,
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value="turtlebot3_burger_example.wbt",
                description="Choose one of the world files from `/jrb_webots/world` directory",
            ),
            webots,
            ros2_supervisor,
            # This action will kill all nodes once the Webots simulation has exited
            launch.actions.RegisterEventHandler(
                event_handler=launch.event_handlers.OnProcessExit(
                    target_action=webots,
                    on_exit=[
                        launch.actions.UnregisterEventHandler(
                            event_handler=reset_handler.event_handler
                        ),
                        launch.actions.EmitEvent(event=launch.events.Shutdown()),
                    ],
                )
            ),
            # Add the reset event handler
            reset_handler,
        ]
        + get_ros2_nodes()
    )
