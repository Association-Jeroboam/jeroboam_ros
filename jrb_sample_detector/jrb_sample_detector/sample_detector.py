from time import sleep
import math
from re import I
import traceback
import rclpy
from rclpy.node import Node

from jrb_msgs.msg import SampleDetected, SampleDetectedArray
from ament_index_python import get_package_share_directory
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped, Pose, Quaternion
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import Image
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf_transformations import (
    concatenate_matrices,
    inverse_matrix,
    translation_matrix,
    quaternion_matrix,
    translation_from_matrix,
    quaternion_from_matrix,
    euler_from_matrix,
    quaternion_from_euler,
)
from copy import copy
from math import radians


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


DATA_PATH = get_package_share_directory("jrb_sample_detector")


def make_marker_msg(id_, stamp, pose, color):
    marker = Marker()

    marker.header.frame_id = "camera_link"
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
            DATA_PATH, "echantillon_rouge.dae"
        )
    elif color == SampleDetected.GREEN:
        marker.mesh_resource = "file://" + os.path.join(
            DATA_PATH, "echantillon_vert.dae"
        )
    elif color == SampleDetected.BLUE:
        marker.mesh_resource = "file://" + os.path.join(
            DATA_PATH, "echantillon_bleu.dae"
        )
    elif color == SampleDetected.ROCK:
        # quat = quaternion_from_euler(math.pi, 0, 0)
        # marker.pose.orientation = Quaternion(**dict(zip(["x", "y", "z", "w"], quat)))

        marker.mesh_resource = "file://" + os.path.join(
            DATA_PATH, "echantillon_surprise.dae"
        )

    marker.mesh_use_embedded_materials = True

    return marker


def opencv_to_ros(vec):
    return np.array([-vec[1], vec[0], vec[2], 1])


DATA_PATH = get_package_share_directory("jrb_sample_detector")


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

        # Tf publisher
        self.tf_broadcaster = TransformBroadcaster(self)

        # Tf subscriber
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cv_bridge = CvBridge()

        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_1000)
        # chargement de la calibration de la camera (faite avec calib_cam.py)
        data = np.load(os.path.join(DATA_PATH, "cam_calib.npz"))
        self.cameraMatrix = data["cameraMatrix"]
        self.distCoeffs = data["distCoeffs"]

        self.arucoParameters = aruco.DetectorParameters_create()
        self.cap = cv2.VideoCapture(os.path.join(DATA_PATH, "outpy.avi"))
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self.tvec0 = np.array([[-0.015, 0.033, 0.317]])
        self.rvecO = np.array([[0, 0, 3.16516633]])

        # calibration en trichant sur la taille des tags :
        self.big_marker_size = 0.050  # en m
        self.small_marker_size = 0.015  # en m

        self.length_of_axis = 0.02  # en m

        # Wait for static transforms
        self.get_logger().info("Wait for static transforms")
        now = self.get_clock().now().to_msg()

        f = self.tf_buffer.wait_for_transform_async("base_link", "left_tag_link", now)
        rclpy.spin_until_future_complete(self, f)

        f = self.tf_buffer.wait_for_transform_async("base_link", "right_tag_link", now)
        rclpy.spin_until_future_complete(self, f)

        f = self.tf_buffer.wait_for_transform_async("base_link", "th_camera_link", now)
        rclpy.spin_until_future_complete(self, f)

        self.get_logger().info("Static transforms found")
        self.base_to_left_tag_tf = self.lookupTransform("base_link", "left_tag_link")
        self.base_to_right_tag_tf = self.lookupTransform("base_link", "right_tag_link")

        # Init camera_link tf equal to th_camera-link
        camera_tf_msg = self.tf_buffer.lookup_transform(
            "base_link", "th_camera_link", now
        )
        camera_tf_msg.child_frame_id = "camera_link"
        self.tf_broadcaster.sendTransform(camera_tf_msg)

        # Camera read thread
        _, self.frame = self.cap.read()
        # self.camera_read_thread = threading.Thread(target=self.camera_read, daemon=True)
        # self.frame_queue = Queue()
        # self.camera_read_thread.start()

        # Publish timer
        analysis_rate = 1 / 3  # seconds
        self.analysis_timer = self.create_timer(analysis_rate, self.timer_callback)

        # cv2.namedWindow("Display", cv2.WND_PROP_FULLSCREEN)
        # cv2.setWindowProperty("Display", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    def cleanup(self):
        if self.camera_read_thread:
            self.camera_read_thread.join()

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

    def camera_read(self):
        return
        # ret, self.frame = self.cap.read()

        while rclpy.ok():
            ret, frame = self.cap.read()

            if not ret:
                continue

            if not self.frame_queue.empty():
                try:
                    self.frame_queue.get_nowait()  # discard previous (unprocessed) frame
                except Queue.Empty:
                    pass

            self.frame_queue.put(frame)
            sleep(1 / 30)

    def get_last_camera_frame(self):
        return self.frame_queue.get()

    def timer_callback(self):
        now = self.get_clock().now().to_msg()

        # frame = self.get_last_camera_frame()
        frame = copy(self.frame)

        img_msg = self.cv_bridge.cv2_to_imgmsg(frame)
        img_msg.header.frame_id = "camera_link"
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
            if (91 in ids) and (92 in ids):
                i_91 = np.where(ids == 91)
                i_92 = np.where(ids == 92)

                if (
                    len(i_91[0]) + len(i_92[0]) == 2
                ):  # pour etre sur qu'il n'y a pas 2 fois le même tag 92 ou 91
                    tvecG_opencv = (
                        tvecs[i_91] * (self.small_marker_size / self.big_marker_size)
                    )[0]

                    tvecD_opencv = (
                        tvecs[i_92] * (self.small_marker_size / self.big_marker_size)
                    )[0]
                    print(tvecD_opencv)

                    tvecG_opencv[2] = 0.3183
                    tvecD_opencv[2] = 0.3183

                    # Construct and publish transform camera -> G
                    self.transform_msg = TransformStamped()
                    self.transform_msg.header.stamp = now
                    self.transform_msg.header.frame_id = "camera_link"
                    self.transform_msg.child_frame_id = "left_tag_camera_link"
                    self.transform_msg.transform.translation.x = tvecG_opencv[0]
                    self.transform_msg.transform.translation.y = tvecG_opencv[1]
                    self.transform_msg.transform.translation.z = tvecG_opencv[2]
                    self.tf_broadcaster.sendTransform(self.transform_msg)

                    # Construct and publish transform camera -> D
                    self.transform_msg = TransformStamped()
                    self.transform_msg.header.stamp = now
                    self.transform_msg.header.frame_id = "camera_link"
                    self.transform_msg.child_frame_id = "right_tag_camera_link"
                    self.transform_msg.transform.translation.x = tvecD_opencv[0]
                    self.transform_msg.transform.translation.y = tvecD_opencv[1]
                    self.transform_msg.transform.translation.z = tvecD_opencv[2]
                    self.tf_broadcaster.sendTransform(self.transform_msg)

                    tvecG = opencv_to_ros(tvecG_opencv)
                    tvecD = opencv_to_ros(tvecD_opencv)

                    camera_pos = 0.5 * (tvecG + tvecD)

                    yaw = math.degrees(
                        np.arctan2(tvecD[0] - tvecG[0], tvecD[1] - tvecG[1])
                    )

                    q = quaternion_from_euler(radians(180), 0, radians(-yaw - 90))
                    self.transform_msg = TransformStamped()
                    self.transform_msg.header.stamp = now
                    self.transform_msg.header.frame_id = "th_camera_link"
                    self.transform_msg.child_frame_id = "camera_link"
                    self.transform_msg.transform.translation.x = -camera_pos[0]
                    self.transform_msg.transform.translation.y = -camera_pos[1]
                    self.transform_msg.transform.translation.z = 0.0
                    self.transform_msg.transform.rotation.x = q[0]
                    self.transform_msg.transform.rotation.y = q[1]
                    self.transform_msg.transform.rotation.z = q[2]
                    self.transform_msg.transform.rotation.w = q[3]
                    self.tf_broadcaster.sendTransform(self.transform_msg)

                    # frame = aruco.drawAxis(
                    #     frame,
                    #     self.cameraMatrix,
                    #     self.distCoeffs,
                    #     self.rvecO,
                    #     self.tvec0,
                    #     self.length_of_axis,
                    # )

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
            msg.header.frame_id = "camera_link"

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
                sample_msg.pose.position.z = 0.332

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
                    msg.header.stamp,
                    sample_msg.pose,
                    sample_msg.type,
                )
                markers_msg.markers.append(marker_msg)

                self.publisher_.publish(msg)
                self.maker_publisher.publish(markers_msg)

        img_msg = self.cv_bridge.cv2_to_imgmsg(frame)
        img_msg.header.frame_id = "camera_link"
        self.image_publisher.publish(img_msg)
        # cv2.imshow('Display', frame)


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
        node.cleanup()


if __name__ == "__main__":
    main()
