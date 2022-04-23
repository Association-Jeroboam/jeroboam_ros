import numpy as np
import cv2 as cv
import glob
import sys
import os
from ament_index_python import get_package_share_directory

# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6 * 7, 3), np.float32)
objp[:, :2] = np.mgrid[0:7, 0:6].T.reshape(-1, 2)
# Arrays to store object points and image points from all the images.
objpoints = []  # 3d point in real world space
imgpoints = []  # 2d points in image plane.

cap = cv.VideoCapture(0)
DATA_PATH = get_package_share_directory("jrb_sample_detector")


def main():
    while True:
        ret, img = cap.read()
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        # Find the chess board corners

        if cv.waitKey(1) & 0xFF == ord("d"):
            ret, corners = cv.findChessboardCorners(gray, (7, 6), None)
            # If found, add object points, image points (after refining them)
            if ret == True:
                objpoints.append(objp)
                corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                imgpoints.append(corners)
                # Draw and display the corners
                cv.drawChessboardCorners(img, (7, 6), corners2, ret)

                ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(
                    objpoints, imgpoints, gray.shape[::-1], None, None
                )

                h, w = img.shape[:2]
                newcameramtx, roi = cv.getOptimalNewCameraMatrix(
                    mtx, dist, (w, h), 1, (w, h)
                )

                file_path = os.path.join(DATA_PATH, "cam_calib.npz")
                np.savez(file_path, cameraMatrix=mtx, distCoeffs=dist)
                print(f"Calibration file saved to {file_path}")

                dst = cv.undistort(img, mtx, dist, None, newcameramtx)

                cv.imshow("dst", dst)
                cv.imshow("origin", img)

        cv.imshow("img_base", img)

        if cv.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv.destroyAllWindows()
