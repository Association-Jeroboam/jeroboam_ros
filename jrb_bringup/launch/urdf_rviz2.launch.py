import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), "/rviz2.launch.py"]),
        launch_arguments={
            "use_sim_time": "true",
            "display_config": os.path.join(
                get_package_share_directory("jrb_description"),
                "rviz",
                "urdf.rviz",
            ),
        }.items(),
    )

    return LaunchDescription([rviz])
