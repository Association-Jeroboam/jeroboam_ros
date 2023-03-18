import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, PathJoinSubstitution
from launch_ros.actions import SetRemap

from launch_ros.actions import Node


def generate_launch_description():
    package_name = "jrb_bringup"

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
        launch_arguments={"use_sim_time": "true", "display_meshes": "false"}.items(),
    )

    twist_mux_params = os.path.join(
        get_package_share_directory("jrb_bringup"), "param", "twist_mux.yaml"
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo_params_path = os.path.join(
        get_package_share_directory(package_name), "param", "gazebo.yaml"
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("gazebo_ros"),
                    "launch",
                    "gazebo.launch.py",
                )
            ]
        ),
        launch_arguments={"params_file": gazebo_params_path}.items(),
    )

    joystick = GroupAction(
        actions=[
            SetRemap(src="/cmd_vel", dst="/cmd_vel_joy"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [ThisLaunchFileDir(), "/joystick.launch.py"]
                ),
                launch_arguments={"use_sim_time": "true"}.items(),
            ),
        ]
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), "/rviz2.launch.py"]),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "robotrouge"],
        output="screen",
    )

    map_manager = Node(
        package="jrb_localization",
        executable="map_manager",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    sample_detector = Node(
        package="jrb_sensors",
        executable="sample_detector",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    go_to_goal = Node(
        package="jrb_control",
        executable="go_to_goal",
        output="screen",
        parameters=[{"use_sim_time": True}],
        remappings=[("/cmd_vel", "/cmd_vel_nav")],
    )

    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        output="screen",
        parameters=[twist_mux_params, {"use_sim_time": True}],
        remappings=[("cmd_vel_out", "/cmd_vel")],
    )

    # Launch them all!
    return LaunchDescription(
        [
            rsp,
            map_manager,
            sample_detector,
            go_to_goal,
            twist_mux,
            joystick,
            rviz,
            gazebo,
            spawn_entity,
        ]
    )
