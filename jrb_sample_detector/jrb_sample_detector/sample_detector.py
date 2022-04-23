import rclpy
from rclpy.node import Node
from rclpy.time import Time

from jrb_msgs.msg import SampleDetected
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Header
from tf_transformations import quaternion_from_matrix
from ament_index_python import get_package_share_directory

import numpy as np
import cv2
import cv2.aruco as aruco
import sys
import math
import os


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
        self.publisher_ = self.create_publisher(SampleDetected, "detected_samples", 10)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

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

        # cv2.namedWindow("Display", cv2.WND_PROP_FULLSCREEN)
        # cv2.setWindowProperty("Display", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    def timer_callback(self):
        ret, frame = self.cap.read()
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
            if (93 in ids) and (92 in ids):
                i_93 = np.where(ids == 93)
                i_92 = np.where(ids == 92)

                if (
                    len(i_93[0]) + len(i_92[0]) == 2
                ):  # pour etre sur qu'il n'y a pas 2 fois le même tag 92 ou 93
                    tvecG = tvecs[i_93] * (
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

                    self.Trc = rotMat.dot(
                        T(self.tvecO[0][0], self.tvecO[0][1], self.tvecO[0][2])
                    )  # Trc : matrice de transformation robot=>camera
                    self.Tcr = np.linalg.inv(
                        self.Trc
                    )  # Tcr : matrice de transformation camera=>robot

                    self.rvecO, _ = cv2.Rodrigues(rotMat[0:3, 0:3])

                    # rvecO=np.array([[0,0,math.atan2(dy,dx)%(2*math.pi)]])

            # frame = aruco.drawAxis(frame, self.cameraMatrix, self.distCoeffs, self.rvecO, self.tvecO, self.length_of_axis)

            # frame = aruco.drawDetectedMarkers(frame, corners, ids)

            rotation_matrix = np.array(
                [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 1]], dtype=float
            )

            msg = SampleDetected()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "robot"

            for i in range(len(tvecs)):
                # frame = aruco.drawAxis(frame, self.cameraMatrix, self.distCoeffs, rvecs[i], tvecs[i], self.length_of_axis)
                if ids[i] == 47:
                    msg.type = SampleDetected.RED
                elif ids[i] == 13:
                    msg.type = SampleDetected.BLUE
                elif ids[i] == 36:
                    msg.type = SampleDetected.GREEN
                    # pass
                elif ids[i] == 17:
                    msg.type = SampleDetected.ROCK
                elif ids[i] == 93:
                    continue
                elif ids[i] == 92:
                    continue
                else:
                    continue

                if self.Tcr.any():
                    xyz = self.Tcr.dot(np.append(tvecs[i], 1))[0:3] * 1000

                    msg.pose.position.x = xyz[0]
                    msg.pose.position.y = xyz[1]
                    msg.pose.position.z = xyz[2]

                    rotation_matrix[:3, :3], _ = cv2.Rodrigues(rvecs[i])
                    q = quaternion_from_matrix(rotation_matrix)
                    msg.pose.orientation.x = q[0]
                    msg.pose.orientation.y = q[1]
                    msg.pose.orientation.z = q[2]
                    msg.pose.orientation.w = q[3]

                    self.publisher_.publish(msg)

        # frame = aruco.drawAxis(frame, self.cameraMatrix, self.distCoeffs, self.rvecO, self.tvecO, self.length_of_axis)
        # cv2.imshow('Display', frame)


def main(args=None):
    rclpy.init(args=args)

    sample_detector = SampleDetector()

    rclpy.spin(sample_detector)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    sample_detector.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
