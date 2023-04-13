from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    joy_params = os.path.join(
        get_package_share_directory("jrb_bringup"), "param", "joystick.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                description="Use simulation clock if true",
                default_value="False",
            ),
            Node(
                package="joy",
                executable="joy_node",
                parameters=[joy_params, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="teleop_twist_joy",
                executable="teleop_node",
                name="teleop_node",
                parameters=[joy_params, {"use_sim_time": use_sim_time}],
            ),
        ]
    )
