import traceback
import rclpy
from rclpy.node import Node

from jrb_msgs.msg import SampleDetected, SampleDetectedArray
from tf_transformations import quaternion_from_matrix
from ament_index_python import get_package_share_directory
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import Image

import numpy as np
import cv2
import cv2.aruco as aruco
import sys
import math
import os
from queue import Queue
import threading
from cv_bridge import CvBridge


def make_marker_msg(id_, stamp, pose, color):
    marker = Marker()

    marker.header.frame_id = "robot"
    marker.header.stamp = stamp

    marker.lifetime = Duration(nanosec=int(1.5 * 1e9))

    # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
    marker.type = 0
    marker.id = id_

    # Set the scale of the marker
    marker.scale.x = 0.3
    marker.scale.y = 0.05
    marker.scale.z = 0.05

    # Set the color
    if color == SampleDetected.RED:
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
    elif color == SampleDetected.GREEN:
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
    elif color == SampleDetected.BLUE:
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0
    elif color == SampleDetected.ROCK:
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

    # Set the pose of the marker
    marker.pose = pose

    return marker


def Rx(angle):
    return np.array(
        [
            [1, 0, 0, 0],
            [0, math.cos(angle), -math.sin(angle), 0],
            [0, math.sin(angle), math.cos(angle), 0],
            [0, 0, 0, 1],
        ]
    )


def Ry(angle):
    return np.array(
        [
            [math.cos(angle), 0, math.sin(angle), 0],
            [0, 1, 0, 0],
            [-math.sin(angle), 0, math.cos(angle), 0],
            [0, 0, 0, 1],
        ]
    )


def Rz(angle):
    return np.array(
        [
            [math.cos(angle), -math.sin(angle), 0, 0],
            [math.sin(angle), math.cos(angle), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ]
    )


def T(x, y, z):
    return np.array([[1, 0, 0, x], [0, 1, 0, y], [0, 0, 1, z], [0, 0, 0, 1]])


def R_euler(a, b, c):
    return (Rz(c).dot(Rx(b))).dot(Rz(a))


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
        self.raw_image_publisher = self.create_publisher(Image, "debug/raw_image", 10)
        self.processed_image_publisher = self.create_publisher(
            Image, "debug/processed_image", 10
        )

        self.tf_camera_robot_broadcaster = TransformBroadcaster(self)

        self.cv_bridge = CvBridge()

        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_1000)
        # chargement de la calibration de la camera (faite avec calib_cam.py)
        data = np.load(os.path.join(DATA_PATH, "cam_calib.npz"))
        self.cameraMatrix = data["cameraMatrix"]
        self.distCoeffs = data["distCoeffs"]

        self.arucoParameters = aruco.DetectorParameters_create()
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self.tvecO = np.array([[-0.015, 0.033, 0.317]])
        self.rvecO = np.array([[0, 0, 3.16516633]])

        # calibration en trichant sur la taille des tags :
        self.big_marker_size = 0.058  # en m
        self.small_marker_size = 0.021  # en m

        self.length_of_axis = 0.02  # en m

        rotMat = R_euler(0, 0, 0.04)
        self.Trc = rotMat.dot(
            T(self.tvecO[0][0], self.tvecO[0][1], self.tvecO[0][2])
        )  # Trc : matrice de transformation robot=>camera
        self.Tcr = np.linalg.inv(
            self.Trc
        )  # Tcr : matrice de transformation camera=>robot

        self.camera_read_thread = threading.Thread(target=self.camera_read, daemon=True)
        self.frame_queue = Queue()
        self.camera_read_thread.start()

        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # cv2.namedWindow("Display", cv2.WND_PROP_FULLSCREEN)
        # cv2.setWindowProperty("Display", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    def cleanup(self):
        if self.camera_read_thread:
            self.camera_read_thread.join()

    def camera_read(self):
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

    def get_last_camera_frame(self):
        return self.frame_queue.get()

    def timer_callback(self):
        frame = self.get_last_camera_frame()

        img_msg = self.cv_bridge.cv2_to_imgmsg(frame)
        img_msg.header.frame_id = "robot"
        # self.raw_image_publisher.publish(img_msg)

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
                    tvecG = tvecs[i_91] * (
                        self.small_marker_size / self.big_marker_size
                    )
                    tvecD = tvecs[i_92] * (
                        self.small_marker_size / self.big_marker_size
                    )

                    self.tvecO = np.mean([tvecG, tvecD], axis=0)
                    # tvecO[0][2]=0.317

                    # angle_x=math.atan2((tvecD[0][2]-tvecG[0][2]),(tvecD[0][1]-tvecG[0][1]))
                    # angle_y=math.atan2((tvecD[0][0]-tvecG[0][0]),(tvecD[0][2]-tvecG[0][2]))
                    angle_z = math.atan2(
                        (tvecD[0][1] - tvecG[0][1]), (tvecD[0][0] - tvecG[0][0])
                    )
                    angle_x = 0
                    angle_y = 0

                    rotMat = R_euler(angle_x, angle_y, angle_z)
                    q = quaternion_from_matrix(rotMat)
                    translation = self.tvecO[0]

                    # Construct and publish transform camera -> robot
                    transform_msg = TransformStamped()
                    transform_msg.header.stamp = self.get_clock().now().to_msg()
                    transform_msg.header.frame_id = "robot"
                    transform_msg.child_frame_id = "camera"
                    transform_msg.transform.translation.x = translation[0]
                    transform_msg.transform.translation.y = translation[1]
                    transform_msg.transform.translation.z = translation[2]
                    transform_msg.transform.rotation.x = q[0]
                    transform_msg.transform.rotation.y = q[1]
                    transform_msg.transform.rotation.z = q[2]
                    transform_msg.transform.rotation.w = q[3]
                    self.tf_camera_robot_broadcaster.sendTransform(transform_msg)

                    self.Trc = rotMat.dot(
                        T(*translation)
                    )  # Trc : matrice de transformation robot=>camera
                    self.Tcr = np.linalg.inv(
                        self.Trc
                    )  # Tcr : matrice de transformation camera=>robot

                    self.rvecO, _ = cv2.Rodrigues(rotMat[0:3, 0:3])

                    # rvecO=np.array([[0,0,math.atan2(dy,dx)%(2*math.pi)]])

            frame = aruco.drawAxis(
                frame,
                self.cameraMatrix,
                self.distCoeffs,
                self.rvecO,
                self.tvecO,
                self.length_of_axis,
            )
            frame = aruco.drawDetectedMarkers(frame, corners, ids)

            rotation_matrix = np.array(
                [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 1]], dtype=float
            )

            msg = SampleDetectedArray()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "robot"

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

                if self.Tcr.any():
                    xyz = self.Tcr.dot(np.append(tvecs[i], 1))[0:3]

                    sample_msg.pose.position.x = xyz[0]
                    sample_msg.pose.position.y = xyz[1]
                    sample_msg.pose.position.z = xyz[2]

                    rotation_matrix[:3, :3], _ = cv2.Rodrigues(rvecs[i])
                    q = quaternion_from_matrix(rotation_matrix)
                    sample_msg.pose.orientation.x = q[0]
                    sample_msg.pose.orientation.y = q[1]
                    sample_msg.pose.orientation.z = q[2]
                    sample_msg.pose.orientation.w = q[3]

                    msg.samples.append(sample_msg)

                    marker_msg = make_marker_msg(
                        i, msg.header.stamp, sample_msg.pose, sample_msg.type
                    )
                    markers_msg.markers.append(marker_msg)

            self.publisher_.publish(msg)
            self.maker_publisher.publish(markers_msg)

        frame = aruco.drawAxis(
            frame,
            self.cameraMatrix,
            self.distCoeffs,
            self.rvecO,
            self.tvecO,
            self.length_of_axis,
        )

        img_msg = self.cv_bridge.cv2_to_imgmsg(frame)
        img_msg.header.frame_id = "robot"
        self.processed_image_publisher.publish(img_msg)
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
