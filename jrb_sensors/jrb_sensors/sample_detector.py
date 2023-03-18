import math
from re import I
import traceback
import rclpy
from rclpy.node import Node

from jrb_msgs.msg import SampleDetected, SampleDetectedArray
from ament_index_python import get_package_share_directory
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf_transformations import (
    concatenate_matrices,
    translation_matrix,
    quaternion_matrix,
    translation_from_matrix,
    euler_from_matrix,
    quaternion_from_euler,
)
from copy import copy
from math import radians
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, FloatingPointRange
from rcl_interfaces.msg import SetParametersResult


import numpy as np
import cv2
import cv2.aruco as aruco
import sys
import math
import os
from queue import Queue
import threading
from cv_bridge import CvBridge
from ament_index_python import get_package_share_directory


DATA_PATH = get_package_share_directory("jrb_sensors")


def make_marker_msg(id_, stamp, pose, color):
    marker = Marker()

    marker.header.frame_id = "camera_link_optical"
    marker.header.stamp = stamp

    marker.lifetime = Duration(nanosec=int(2 * 1e9))

    marker.type = Marker.MESH_RESOURCE
    marker.action = Marker.ADD
    marker.id = id_

    marker.scale.x = 1.0
    marker.scale.y = 1.0
    marker.scale.z = 1.0

    # Set the pose of the marker
    marker.pose = pose

    # Set the color
    if color == SampleDetected.RED:
        marker.mesh_resource = "file://" + os.path.join(
            DATA_PATH, "meshes/echantillon_rouge.dae"
        )
    elif color == SampleDetected.GREEN:
        marker.mesh_resource = "file://" + os.path.join(
            DATA_PATH, "meshes/echantillon_vert.dae"
        )
    elif color == SampleDetected.BLUE:
        marker.mesh_resource = "file://" + os.path.join(
            DATA_PATH, "meshes/echantillon_bleu.dae"
        )
    elif color == SampleDetected.ROCK:
        marker.mesh_resource = "file://" + os.path.join(
            DATA_PATH, "meshes/echantillon_surprise.dae"
        )

    marker.mesh_use_embedded_materials = True

    return marker


def opencv_to_ros(vec):
    return np.array([-vec[1], vec[0], vec[2]])


DATA_PATH = get_package_share_directory("jrb_sensors")


class SampleDetector(Node):
    def __init__(self):
        super().__init__("sample_detector")
        self.get_logger().info("init")

        self.publisher_ = self.create_publisher(
            SampleDetectedArray, "sample_detected", 10
        )
        self.maker_publisher = self.create_publisher(
            MarkerArray, "debug/sample_detected", 10
        )
        self.image_publisher = self.create_publisher(Image, "debug/image", 10)
        self.image_subscriber = self.create_subscription(
            CompressedImage,
            "/camera/image_raw/compressed",
            self.on_image_compressed,
            10,
        )
        self.camera_info_subscriber = self.create_subscription(
            CameraInfo, "/camera/camera_info", self.on_camera_info, 10
        )

        # Tf publisher
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        # Tf subscriber
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cv_bridge = CvBridge()
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_1000)
        self.cameraMatrix = None
        self.distCoeffs = None
        self.arucoParameters = aruco.DetectorParameters_create()

        # calibration en trichant sur la taille des tags :
        self.big_marker_size = 0.050  # en m
        self.small_marker_size = 0.015  # en m

        self.length_of_axis = 0.02  # en m

        # Wait for static transforms
        self.get_logger().info("Wait for static transforms")
        now = self.get_clock().now().to_msg()

        f = self.tf_buffer.wait_for_transform_async("chassis", "camera_link", now)
        rclpy.spin_until_future_complete(self, f)

        self.get_logger().info("Static transforms found")

        self.add_on_set_parameters_callback(self.on_update_parameters)

    def on_update_parameters(self, params):
        self.get_logger().info("Params updated")

        return SetParametersResult(successful=True)

    def lookupTransform(
        self, target_frame, source_frame, time=rclpy.time.Time().to_msg()
    ):
        transform_msg = self.tf_buffer.lookup_transform(
            target_frame, source_frame, time
        )
        trans = np.array(
            [
                transform_msg.transform.translation.x,
                transform_msg.transform.translation.y,
                transform_msg.transform.translation.z,
            ]
        )
        rot = np.array(
            [
                transform_msg.transform.rotation.x,
                transform_msg.transform.rotation.y,
                transform_msg.transform.rotation.z,
                transform_msg.transform.rotation.w,
            ]
        )
        transform = concatenate_matrices(
            translation_matrix(trans), quaternion_matrix(rot)
        )

        return transform

    def on_camera_info(self, msg):
        self.get_logger().info("Got camera info, unsubscribing")
        self.cameraMatrix = np.array(msg.k).reshape(3, 3)
        self.distCoeffs = np.array(msg.d).reshape(1, 5)
        self.destroy_subscription(self.camera_info_subscriber)

    def on_image_compressed(self, msg):
        if self.cameraMatrix is None or self.distCoeffs is None:
            self.get_logger().warn("No CameraInfo yet, discard frame")
            return

        now = self.get_clock().now().to_msg()

        frame = self.cv_bridge.compressed_imgmsg_to_cv2(msg)

        # img_msg = self.cv_bridge.cv2_to_imgmsg(frame)
        # img_msg.header.frame_id = "camera_link"
        # self.image_publisher.publish(img_msg)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.arucoParameters
        )

        if (
            type(ids).__module__ == np.__name__
        ):  # on vérifie que ids est un numpy array. C'est le cas uniquement si il y a au moins en tag de détecté.
            rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(
                corners, self.big_marker_size, self.cameraMatrix, self.distCoeffs
            )

            # Draw origin
            frame = aruco.drawAxis(
                frame,
                self.cameraMatrix,
                self.distCoeffs,
                np.array([[0.0, 0.0, 0.0]]),
                np.array([[0.0, 0.0, 0.0]]),
                self.length_of_axis,
            )

            frame = aruco.drawDetectedMarkers(frame, corners, ids)

            msg = SampleDetectedArray()
            msg.header.stamp = now
            msg.header.frame_id = "camera_link_optical"

            markers_msg = MarkerArray()

            for i in range(len(tvecs)):
                sample_msg = SampleDetected()

                frame = aruco.drawAxis(
                    frame,
                    self.cameraMatrix,
                    self.distCoeffs,
                    rvecs[i],
                    tvecs[i],
                    self.length_of_axis,
                )
                if ids[i] == 47:
                    sample_msg.type = SampleDetected.RED
                elif ids[i] == 13:
                    sample_msg.type = SampleDetected.BLUE
                elif ids[i] == 36:
                    sample_msg.type = SampleDetected.GREEN
                    # pass
                elif ids[i] == 17:
                    sample_msg.type = SampleDetected.ROCK
                elif ids[i] == 91:
                    continue
                elif ids[i] == 92:
                    continue
                else:
                    continue

                xyz = tvecs[i][0]

                sample_msg.pose.position.x = xyz[0]
                sample_msg.pose.position.y = xyz[1]
                sample_msg.pose.position.z = 0.355

                rotation_matrix = np.identity(4)
                rotation_matrix[:3, :3], _ = cv2.Rodrigues(np.array([rvecs[i][0][:3]]))
                _, _, yaw = euler_from_matrix(rotation_matrix)
                q = quaternion_from_euler(0, 0, yaw)
                sample_msg.pose.orientation.x = q[0]
                sample_msg.pose.orientation.y = q[1]
                sample_msg.pose.orientation.z = q[2]
                sample_msg.pose.orientation.w = q[3]

                msg.samples.append(sample_msg)

                marker_msg = make_marker_msg(
                    len(markers_msg.markers),
                    now,
                    sample_msg.pose,
                    sample_msg.type,
                )
                markers_msg.markers.append(marker_msg)

            self.publisher_.publish(msg)
            self.maker_publisher.publish(markers_msg)

        img_msg = self.cv_bridge.cv2_to_imgmsg(frame)
        img_msg.header.frame_id = "camera_link_optical"
        self.image_publisher.publish(img_msg)


def main(args=None):
    rclpy.init(args=args)

    node = SampleDetector()

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
