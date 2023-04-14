import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, PathJoinSubstitution
from launch_ros.actions import SetRemap
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    package_name = "jrb_bringup"

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
        launch_arguments={"use_sim_time": "true", "display_meshes": "false"}.items(),
    )

    twist_mux_params = os.path.join(
        get_package_share_directory("jrb_bringup"), "param", "twist_mux.yaml"
    )

    marker_publisher_params = os.path.join(
        get_package_share_directory("jrb_bringup"), "param", "marker_publisher.yaml"
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
        remappings=[("/cmd_vel", "/cmd_vel_nav"), ("/odometry", "/odom")],
    )

    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        output="screen",
        parameters=[twist_mux_params, {"use_sim_time": True}],
        remappings=[("cmd_vel_out", "/cmd_vel")],
    )

    marker_publisher = Node(
        package="jrb_strategy2",
        executable="marker_publisher",
        output="screen",
        parameters=[marker_publisher_params, {"use_sim_time": True}],
        remappings=[("markers", "/debug/table_mesh")],
    )

    static_transform_broadcaster = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        parameters=[marker_publisher_params, {"use_sim_time": True}],
        condition=UnlessCondition(global_localization)
    )

    # Launch them all!
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "global_localization",
                description="Use a global localization node such as amcl to have the tf map->odom",
                default_value="False"
            ),
            rsp,
            static_transform_broadcaster,
            sample_detector,
            go_to_goal,
            twist_mux,
            joystick,
            marker_publisher,
            rviz,
            gazebo,
            spawn_entity,
        ]
    )
