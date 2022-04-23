import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

rviz_config_path = os.path.join(
    get_package_share_directory("jrb_sample_detector"),
    "sample_detector.rviz",
)
print(rviz_config_path)


def generate_launch_description():
    rviz_config_path = os.path.join(
        get_package_share_directory("jrb_sample_detector"),
        "sample_detector.rviz",
    )

    return LaunchDescription(
        [
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config_path],
                output="screen",
            ),
        ]
    )
