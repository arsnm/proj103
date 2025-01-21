# inspired from https://github.com/GSNCodes/ArUCo-Markers-Pose-Estimation-Generation-Python/blob/main/pose_estimation.py
"""
Sample Usage:-
python pose_estimation.py --calibration camera_calibration.npz --type DICT_6X6_50
"""

import numpy as np
import cv2
import sys
from src.utils.aruco_markers import ARUCO_DICT
from src.config import ArucoConfig
import argparse
import time


def pose_esitmation(
    frame, aruco_dict_type, matrix_coefficients, distortion_coefficients, marker_size
):
    """
    frame - Frame from the video stream
    matrix_coefficients - Intrinsic matrix of the calibrated camera
    distortion_coefficients - Distortion coefficients associated with your camera

    return:-
    frame - The frame with the axis drawn on it
    """

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    arucoDict = cv2.aruco.getPredefinedDictionary(aruco_dict_type)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(arucoDict, parameters)

    corners, ids, rejected_img_points = detector.detectMarkers(gray)

    if len(corners) > 0:
        for i in range(0, len(ids)):
            # Estimate pose of each marker and return the values rvec and tvec---(different from those of camera coefficients)
            rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(
                corners[i], marker_size, matrix_coefficients, distortion_coefficients
            )
            # Draw a square around the markers
            cv2.aruco.drawDetectedMarkers(frame, corners)

            # Draw Axis
            cv2.drawFrameAxes(
                frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.01
            )

            # Calculate distance in cm (tvec gives the translation vector in meters)
            distance = np.linalg.norm(tvec[0][0]) * 100

            # Calculate angle in degrees between the marker and the optical axis
            angle_to_optical_axis = np.arctan2(tvec[0][0][0], tvec[0][0][2]) * (
                180 / np.pi
            )

            # Calculate the angle between the optical axis and the z-axis of the ArUco marker
            rotation_matrix, _ = cv2.Rodrigues(rvec[0])
            z_axis = rotation_matrix[
                :, 2
            ]  # Z-axis of the ArUco marker in camera coordinates
            optical_axis = np.array([0, 0, 1])  # Optical axis of the camera
            dot_product = np.dot(z_axis, optical_axis)
            magnitude_z = np.linalg.norm(z_axis)
            magnitude_optical = np.linalg.norm(optical_axis)
            angle_to_z_axis = np.arccos(
                dot_product / (magnitude_z * magnitude_optical)
            ) * (180 / np.pi)

            # Calculate the absolute facing angle of the camera based on the ArUco marker's facing direction
            aruco_facing_angle = (
                180  # Assume the ArUco marker faces south (180 degrees)
            )
            camera_facing_angle = (aruco_facing_angle + angle_to_optical_axis) % 360

            # Display distance, angles, and camera facing angle on the frame
            center = tuple(corners[i][0].mean(axis=0).astype(int))
            cv2.putText(
                frame,
                f"Dist: {distance:.2f} cm",
                (center[0], center[1] - 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2,
            )
            cv2.putText(
                frame,
                f"Angle to optical axis: {angle_to_optical_axis:.2f} deg",
                (center[0], center[1] - 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2,
            )
            cv2.putText(
                frame,
                f"Angle to Z-axis: {angle_to_z_axis:.2f} deg",
                (center[0], center[1] - 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2,
            )
            cv2.putText(
                frame,
                f"Camera Facing: {camera_facing_angle:.2f} deg",
                (center[0], center[1] - 80),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2,
            )
    return frame


def pose_estimation_solve_pnp(
    frame, aruco_dict_type, matrix_coefficients, distortion_coefficients
):
    """
    frame - Frame from the video stream
    matrix_coefficients - Intrinsic matrix of the calibrated camera
    distortion_coefficients - Distortion coefficients associated with your camera

    return:-
    {"frame": frame
     "markers": [(id, x, y, angle_to_z_axis), ...]}
     "matrix" : [(id, tvec, rvec)]
    """

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    arucoDict = cv2.aruco.getPredefinedDictionary(aruco_dict_type)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(arucoDict, parameters)

    corners, ids, rejected_img_points = detector.detectMarkers(gray)

    poses = {}  # marker: [(id, x, y, angle_with_optical_axis), ...]
    poses["markers"] = []
    poses["matrix"] = []

    if len(corners) > 0:
        for i in range(0, len(ids)):

            if ids[i][0] in range(1, 5):
                marker_size = ArucoConfig.POSITION_SIZE.value
            elif ids[i][0] == 0:
                marker_size = ArucoConfig.HINT_SIZE.value
            elif ids[i][0] in range(5, 17):
                marker_size = ArucoConfig.FLAG_SIZE.value
            else:
                marker_size = ArucoConfig.DEFAULT_SIZE.value
            # Define the 3D points of the marker in world space
            marker_3d_points = np.array(
                [
                    [-marker_size / 2, -marker_size / 2, 0],
                    [marker_size / 2, -marker_size / 2, 0],
                    [marker_size / 2, marker_size / 2, 0],
                    [-marker_size / 2, marker_size / 2, 0],
                ]
            )

            # Corresponding 2D image points of the marker corners
            image_points = np.array(corners[i][0], dtype="float32")

            # Use solvePnP to estimate the pose
            success, rvec, tvec = cv2.solvePnP(
                marker_3d_points,
                image_points,
                matrix_coefficients,
                distortion_coefficients,
            )

            if success:
                # Draw a square around the markers
                cv2.aruco.drawDetectedMarkers(frame, corners)

                # Draw the 3D axis
                cv2.drawFrameAxes(
                    frame,
                    matrix_coefficients,
                    distortion_coefficients,
                    rvec,
                    tvec,
                    marker_size / 2,
                )

                x, _, y = tvec.flatten()  # not using the same coordinates system
                distance = np.sqrt(x**2 + y**2)

                # Calculate angle in degrees between the marker and the optical axis
                # angle_to_optical_axis = np.arctan2(tvec[0], tvec[2]) * (180 / np.pi)

                # Calculate the angle between the optical axis and the z-axis of the ArUco marker
                rotation_matrix, _ = cv2.Rodrigues(rvec)
                z_axis = rotation_matrix[
                    :, 2
                ]  # Z-axis of the ArUco marker in camera coordinates
                optical_axis = np.array([0, 0, 1])  # Optical axis of the camera
                dot_product = np.dot(z_axis, optical_axis)
                angle_to_z_axis = np.arccos(dot_product)
                cross_product = np.cross(z_axis, optical_axis)

                if cross_product[1] < 0:
                    angle_to_z_axis *= -1

                # # Calculate the absolute facing angle of the camera based on the ArUco marker's facing direction
                # aruco_facing_angle = (
                #     180  # Assume the ArUco marker faces south (180 degrees)
                # )
                # camera_facing_angle = (aruco_facing_angle + angle_to_optical_axis) % 360

                # Display distance, angles, and camera facing angle on the frame
                center = tuple(corners[i][0].mean(axis=0).astype(int))
                cv2.putText(
                    frame,
                    f"Id: {ids[i]}, Dist: {distance * 100:.2f}cm",
                    (center[0], center[1] - 20),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2,
                )
                # cv2.putText(
                #     frame,
                #     f"Angle to optical axis: {angle_to_optical_axis.item():.2f} deg",
                #     (center[0], center[1] - 40),
                #     cv2.FONT_HERSHEY_SIMPLEX,
                #     0.5,
                #     (0, 255, 0),
                #     2,
                # )
                cv2.putText(
                    frame,
                    f"Angle Z-axis: {angle_to_z_axis * 180 / np.pi:.2f}deg",
                    (center[0], center[1] - 60),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2,
                )
                # cv2.putText(
                #     frame,
                #     f"Camera Facing: {camera_facing_angle.item():.2f} deg",
                #     (center[0], center[1] - 80),
                #     cv2.FONT_HERSHEY_SIMPLEX,
                #     0.5,
                #     (0, 255, 0),
                #     2,
                # )
                poses["markers"].append((ids[i], x, y, angle_to_z_axis))
                poses["matrix"].append((ids[i], tvec, rvec))

    poses["frame"] = frame
    return poses


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "-c",
        "--calibration",
        required=True,
        help="Path to calibration file (numpy file)",
    )
    ap.add_argument(
        "-t",
        "--type",
        type=str,
        default="DICT_6X6_50",
        help="Type of ArUCo tag to detect",
    )

    args = vars(ap.parse_args())

    if ARUCO_DICT.get(args["type"], None) is None:
        print(f"ArUCo tag type '{args['type']}' is not supported")
        sys.exit(0)

    aruco_dict_type = ARUCO_DICT[args["type"]]
    calibraton_file_path = args["calibration"]

    calibration_file = np.load(calibraton_file_path)

    k = calibration_file["camera_matrix"]
    d = calibration_file["dist_coeffs"]

    video = cv2.VideoCapture(0)
    video.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    video.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    video.set(cv2.CAP_PROP_FPS, 30)
    time.sleep(2.0)

    while True:
        ret, frame = video.read()

        if not ret:
            continue

        output = pose_estimation_solve_pnp(frame, aruco_dict_type, k, d)

        cv2.imshow("Estimated Pose", output["frame"])

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break

    video.release()
    cv2.destroyAllWindows()
