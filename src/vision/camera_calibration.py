# taken from https://github.com/GSNCodes/ArUCo-Markers-Pose-Estimation-Generation-Python/blob/main/calibration.py
import numpy as np
import cv2
import os
import argparse
from src.config import CalibrationConfig


def calibrate(dirpath, square_size, width, height, visualize=False):
    """Apply camera calibration operation for images in the given directory path."""

    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(width-1,height-1,0)
    objp = np.zeros((height * width, 3), np.float32)
    objp[:, :2] = np.mgrid[0:width, 0:height].T.reshape(-1, 2)
    objp *= square_size

    # Arrays to store object points and image points from all the images.
    objpoints = []  # 3d point in real world space
    imgpoints = []  # 2d points in image plane.

    images = os.listdir(dirpath)

    for fname in images:
        filepath = os.path.join(dirpath, fname)
        img = cv2.imread(filepath)

        if img is None:
            print(f"Warning: Could not read image {filepath}. Skipping.")
            continue

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (width, height), None)

        # If found, add object points, image points (after refining them)
        if ret:
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)

            # Draw and display the corners
            img = cv2.drawChessboardCorners(img, (width, height), corners2, ret)

        if visualize:
            cv2.imshow("img", img)
            cv2.waitKey(500)  # Show each image for 500 ms

    cv2.destroyAllWindows()

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, gray.shape[::-1], None, None
    )

    mean_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
        mean_error += error

    return [ret, mtx, dist, rvecs, tvecs, mean_error / len(objpoints)]


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "-d",
        "--dir",
        default=CalibrationConfig.DIR_CAL_IMAGE.value,
        help=f"Path to folder containing checkerboard images for calibration (default={CalibrationConfig.DIR_CAL_IMAGE.value})",
    )
    ap.add_argument(
        "-w",
        "--width",
        type=int,
        default=CalibrationConfig.CHESS_WIDTH.value,
        help=f"Width of checkerboard (default={CalibrationConfig.CHESS_WIDTH.value})",
    )
    ap.add_argument(
        "-t",
        "--height",
        type=int,
        default=CalibrationConfig.CHESS_HEIGHT.value,
        help=f"Width of checkerboard (default={CalibrationConfig.CHESS_HEIGHT.value})",
    )
    ap.add_argument(
        "-s",
        "--square_size",
        type=float,
        default=CalibrationConfig.CHESS_SIZE.value,
        help=f"Length of one edge (in meters)(default={CalibrationConfig.CHESS_SIZE.value})",
    )
    ap.add_argument(
        "-v",
        "--visualize",
        action="store_true",
        help="Visualize each checkerboard image",
    )
    ap.add_argument(
        "-o",
        "--output",
        type=str,
        default=CalibrationConfig.CAL_FILE.value,
        help=f"File in which the calibration will be saved (default={CalibrationConfig.CAL_FILE.value})",
    )

    args = ap.parse_args()

    ret, mtx, dist, rvecs, tvecs, error = calibrate(
        dirpath=args.dir,
        square_size=args.square_size,
        width=args.width,
        height=args.height,
        visualize=args.visualize,
    )

    print("Camera Matrix:\n", mtx)
    print("Distortion Coefficients:\n", dist)
    print("Total Error:", error)

    np.savez(args.output, camera_matrix=mtx, dist_coeffs=dist)
