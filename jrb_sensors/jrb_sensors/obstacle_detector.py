#!/usr/bin/env python3

from math import radians, pi
import traceback
import rclpy
from rclpy.node import Node
from ament_index_python import get_package_share_directory
from visualization_msgs.msg import Marker, MarkerArray
from rcl_interfaces.msg import (
    ParameterDescriptor,
    ParameterType,
    FloatingPointRange,
    IntegerRange,
)
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from builtin_interfaces.msg import Duration
import numpy as np
from jrb_sensors.ClusterBuffer import ClusterBuffer
from jrb_sensors.TfMessageFilters import TfMessageFilter
import message_filters
from geometry_msgs.msg import PoseArray, Pose, TransformStamped, Transform
from rcl_interfaces.msg import SetParametersResult
from tf_transformations import (
    concatenate_matrices,
    translation_matrix,
    quaternion_matrix,
    euler_from_matrix,
    rotation_from_matrix,
    quaternion_from_euler,
)
from rclpy.qos import QoSReliabilityPolicy, QoSProfile


def sgn(number:float) -> int:
    if number < 0.0:
        return -1

    return 1

def normalize_angle(angle):
    # return angle
    normalized_angle = angle % (2 * pi)    # Normalizes from 0 to 2*pi
    if normalized_angle > pi:              # Shifts to -pi to pi
        normalized_angle -= 2 * pi
    return normalized_angle

DATA_PATH = get_package_share_directory("jrb_sensors")
DETECT_ANGLE = 90  # deg
LASER_LINK_ANGLE_TO_ROBOT = 90  # deg
MIN_ANGLE = normalize_angle(radians(LASER_LINK_ANGLE_TO_ROBOT - DETECT_ANGLE / 2.0))
MAX_ANGLE = normalize_angle(radians(LASER_LINK_ANGLE_TO_ROBOT + DETECT_ANGLE / 2.0))
print("min", MIN_ANGLE, " max", MAX_ANGLE)
MIN_ANGLE_REVERSE = normalize_angle(MIN_ANGLE + pi)
MAX_ANGLE_REVERSE = normalize_angle(MAX_ANGLE + pi)

    


def transform_msg_to_matrix(msg: Transform):
    trans = np.array(
        [
            msg.transform.translation.x,
            msg.transform.translation.y,
            msg.transform.translation.z,
        ]
    )
    rot = np.array(
        [
            msg.transform.rotation.x,
            msg.transform.rotation.y,
            msg.transform.rotation.z,
            msg.transform.rotation.w,
        ]
    )

    return concatenate_matrices(translation_matrix(trans), quaternion_matrix(rot))


DATA_PATH = get_package_share_directory("jrb_sensors")


class ObstacleDetector(Node):
    def __init__(self):
        super().__init__("obstacle_detector")
        self.get_logger().info(f"{self.get_name()} started")

        lidar_qos_profile = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy(QoSReliabilityPolicy.BEST_EFFORT),
        )

        self.maker_publisher = self.create_publisher(
            MarkerArray, "debug/obstacle_detected", 10
        )
        self.scan_filtered_publisher = self.create_publisher(
            LaserScan, "debug/scan_filtered", lidar_qos_profile
        )
        self.obstacle_detected_publisher = self.create_publisher(
            PoseArray, "obstacle_detected", 10
        )

        self.odom_sub = self.create_subscription(Odometry, "/odometry", self.on_odometry, 1)
        self._lidar_subscriber = message_filters.Subscriber(
            self, LaserScan, "/scan", qos_profile=lidar_qos_profile
        )
        self.lidar_subscriber = TfMessageFilter(
            self._lidar_subscriber, "odom", "laser_link", queue_size=10
        )
        self.lidar_subscriber.registerCallback(self.on_scan)

        self.declare_parameter(
            "min_distance",
            0.1,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                floating_point_range=[
                    FloatingPointRange(from_value=0.1, to_value=12.0, step=0.1)
                ],
            ),
        )

        self.declare_parameter(
            "max_distance",
            0.5,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                floating_point_range=[
                    FloatingPointRange(from_value=0.1, to_value=12.0, step=0.1)
                ],
            ),
        )

        self.declare_parameter(
            "cluster_size",
            5,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                integer_range=[IntegerRange(from_value=2, to_value=20, step=1)],
            ),
        )

        self.declare_parameter(
            "tolerance",
            2,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                integer_range=[IntegerRange(from_value=1, to_value=19, step=1)],
            ),
        )

        self.min_distance = self.get_parameter("min_distance").value
        self.max_distance = self.get_parameter("max_distance").value
        self.cluster_size = self.get_parameter("cluster_size").value
        self.tolerance = self.get_parameter("tolerance").value

        self.__cos_sin_map = np.array([[]])
        self.cluster_buffer = ClusterBuffer(
            maxlen=self.cluster_size, clusterlen=self.cluster_size - self.tolerance
        )

        self.pose_array_msg = PoseArray()
        self.pose_array_msg.header.frame_id = "map"

        self.marker_array_msg = MarkerArray()

        self.linear_velocity = 0.0

        self.add_on_set_parameters_callback(self.on_update_parameters)

    def on_update_parameters(self, params):
        self.get_logger().info("Params updated")

        for param in params:
            if param.name == "min_distance":
                self.min_distance = param.value
            elif param.name == "max_distance":
                self.max_distance = param.value
            elif param.name == "cluster_size":
                self.cluster_size = param.value
                self.cluster_buffer = ClusterBuffer(
                    maxlen=self.cluster_size,
                    clusterlen=self.cluster_size - self.tolerance,
                )
            elif param.name == "tolerance":
                self.tolerance = param.value
                self.cluster_buffer = ClusterBuffer(
                    maxlen=self.cluster_size,
                    clusterlen=self.cluster_size - self.tolerance,
                )

        return SetParametersResult(successful=True)

    def add_marker(self, id_, stamp, pose):
        marker = Marker()

        marker.header.frame_id = "odom"
        marker.header.stamp = stamp

        marker.lifetime = Duration(nanosec=int(1 / 8 * 1e9))

        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.id = id_

        marker.scale.x = 0.01
        marker.scale.y = 0.1
        marker.scale.z = 0.4

        # Set the pose of the marker
        marker.pose = pose
        marker.pose.position.z -= marker.scale.z / 2

        # Set the color
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0

        self.marker_array_msg.markers.append(marker)

    def on_odometry(self, msg:Odometry):
        # self.linear_velocity = msg.twist.twist.linear.x
        self.linear_velocity = -1

    def on_scan(self, msg: LaserScan, transform_msg: TransformStamped):
        N = len(msg.ranges)
        transform = transform_msg_to_matrix(transform_msg)
        ranges = np.array(msg.ranges)
        # intensities = np.array(msg.intensities)

        if (
            self.__cos_sin_map.shape[1] != N
            or self.__angle_min != msg.angle_min
            or self.__angle_max != msg.angle_max
        ):
            self.get_logger().debug("No precomputed map given. Computing one.")

            self.__angle_min = msg.angle_min
            self.__angle_max = msg.angle_max

            angles = msg.angle_min + np.arange(N) * msg.angle_increment
            self.__cos_sin_map = np.array([np.cos(angles), np.sin(angles)]).transpose()

        self.pose_array_msg.poses = []
        self.pose_array_msg.header.stamp = msg.header.stamp
        self.marker_array_msg.markers = []
        print((-pi, MIN_ANGLE_REVERSE, MAX_ANGLE_REVERSE, pi))

        for i, (cos_sin, angle, range) in enumerate(
            zip(self.__cos_sin_map, angles, ranges)
        ):
            if (
                # not (self.min_distance <= range <= self.max_distance)
                False
                or (sgn(self.linear_velocity) > 0 and not (MIN_ANGLE <= angle <= MAX_ANGLE))
                or (sgn(self.linear_velocity) < 0 and not (MIN_ANGLE_REVERSE <= angle <= MAX_ANGLE_REVERSE))
            ):
                self.cluster_buffer.add_pose(pose=None, valid=False)
                msg.ranges[i] = float("inf")
                continue

            point_in_laser_link = range * cos_sin
            x_map, y_map, z_map, _ = transform.dot(
                np.concatenate((point_in_laser_link, [0, 1]))
            ).tolist()
            _, _, yaw_map = euler_from_matrix(transform) + angle
            q_map = quaternion_from_euler(0, 0, yaw_map)

            if not (0 <= x_map <= 2000) or not (0 <= y_map <= 3000):
                self.cluster_buffer.add_pose(pose=None, valid=False)
                msg.ranges[i] = float("inf")
                continue

            self.cluster_buffer.add_pose(pose=(x_map, y_map, yaw_map), valid=True)

            cluster_pos = self.cluster_buffer.get_cluster_pose()
            if cluster_pos is not None:
                pose = Pose()
                pose.position.x = x_map
                pose.position.y = y_map
                pose.position.z = z_map
                pose.orientation.x = q_map[0]
                pose.orientation.y = q_map[1]
                pose.orientation.z = q_map[2]
                pose.orientation.w = q_map[3]

                self.pose_array_msg.poses.append(pose)
                self.add_marker(i, msg.header.stamp, pose)

        self.obstacle_detected_publisher.publish(self.pose_array_msg)
        self.maker_publisher.publish(self.marker_array_msg)
        self.scan_filtered_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = ObstacleDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped cleanly")
    except Exception:
        print("Error while stopping the node")
        print(traceback.format_exc())
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
