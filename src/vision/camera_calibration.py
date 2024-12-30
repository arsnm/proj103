import numpy as np
import cv2
import os
import argparse


class CameraCalibrator:
    def __init__(
        self, chessboard_size=(7, 7), square_size=0.025, visualize=False, dirpath=None
    ):
        self.chessboard_size = chessboard_size
        self.square_size = square_size  # Size of a square in meters

        # Prepare object points (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        self.objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
        self.objp[:, :2] = np.mgrid[
            0 : chessboard_size[0], 0 : chessboard_size[1]
        ].T.reshape(-1, 2)
        self.objp = self.objp * square_size

        # Arrays to store object points and image points
        self.objpoints = []  # 3d points in real world space
        self.imgpoints = []  # 2d points in image plane

        self.dirpath = dirpath

        self.visualize = visualize

        # Calibration results
        self.camera_matrix = None
        self.dist_coeffs = None
        self.rvecs = None
        self.tvecs = None
        self.error = None

    def find_chessboard_corners(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
        ret, corners = cv2.findChessboardCorners(gray, self.chessboard_size, flags)

        if ret:
            criteria = (
                cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                30,
                0.001,
            )
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

            img = cv2.drawChessboardCorners(frame, self.chessboard_size, corners2, ret)
            self.objpoints.append(self.objp)
            self.imgpoints.append(corners2)

            if self.visualize:
                cv2.imshow("img", img)
                cv2.waitKey(0)

    def calibrate(self):
        images = os.listdir(self.dirpath)
        img_size = None

        for fname in images:
            img = cv2.imread(os.path.join(self.dirpath, fname))
            self.find_chessboard_corners(img)
            img_size = img.shape[::-1]

        if len(self.objpoints) < 5:
            raise ValueError("Need at least 5 valid chessboard images for calibration")

        if img_size is not None:
            ret, self.camera_matrix, self.dist_coeffs, self.rvecs, self.tvecs = (
                cv2.calibrateCamera(
                    self.objpoints, self.imgpoints, img_size, None, None
                )
            )

            mean_error = 0
            for i in range(len(self.objpoints)):
                imgpoints2, _ = cv2.projectPoints(
                    self.objpoints[i],
                    self.rvecs[i],
                    self.tvecs[i],
                    self.camera_matrix,
                    self.dist_coeffs,
                )
                error = cv2.norm(self.imgpoints[i], imgpoints2, cv2.NORM_L2) / len(
                    imgpoints2
                )
                mean_error += error
            self.error = mean_error / len(self.objpoints)

    def save_calibration(self, filename):
        if self.camera_matrix is None or self.dist_coeffs is None:
            raise ValueError("Camera not calibrated yet")

        np.savez(
            filename,
            camera_matrix=self.camera_matrix,
            dist_coeffs=self.dist_coeffs,
            error=self.error,
        )
        print(f"Calibration saved to {filename}")
        print(f"Reprojection error: {self.error}")
        print("\nCamera matrix:")
        print(self.camera_matrix)
        print("\nDistortion coefficients:")
        print(self.dist_coeffs)


def main():
    """
    Main function for CLI usage.
    """
    parser = argparse.ArgumentParser(description="Camera Calibration Script")
    parser.add_argument(
        "-o",
        "--output",
        type=str,
        default="camera_calibration.npz",
        help="Output file for calibration data",
    )
    parser.add_argument(
        "-x",
        "--squares-x",
        type=int,
        default=7,
        help="Number of inner corners on x axis of the chessboard",
    )
    parser.add_argument(
        "-y",
        "--squares-y",
        type=int,
        default=7,
        help="Number of inner corners on y axis of the chessboard",
    )
    parser.add_argument(
        "-s",
        "--square-size",
        type=float,
        default=0.025,
        help="Size of a chessboard square in meters",
    )
    parser.add_argument(
        "-d" "--dir-path",
        type=str,
        help="Directory from which/where to load/save the images",
    )
    parser.add_argument(
        "-v" "--visualize",
        type=str,
        default="False",
        help="To visualize each checkboard image",
    )

    args = parser.parse_args()

    if args.visualize.lower() == "true":
        visualize = True
    else:
        visualize = False

    calibrator = CameraCalibrator(
        chessboard_size=(args.squares_x, args.squares_y),
        square_size=args.square_size,
        visualize=visualize,
        dirpath=args.dir_path,
    )

    img_counter = 0

    calibrator.calibrate()

    print(f"Calibration error: {calibrator.error}")
    print(f"Camera matrix: {calibrator.camera_matrix}")
    print(f"Camera matrix: {calibrator.dist_coeffs}")

    calibrator.save_calibration(args.output)


if __name__ == "__main__":
    main()
