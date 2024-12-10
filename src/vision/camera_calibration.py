import numpy as np
import cv2
import argparse


class CameraCalibrator:
    def __init__(self, chessboard_size=(9, 6), square_size=0.025):
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
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

            cv2.drawChessboardCorners(frame, self.chessboard_size, corners2, ret)
            self.objpoints.append(self.objp)
            self.imgpoints.append(corners2)

        return ret, frame

    def calibrate(self, img_size):
        if len(self.objpoints) < 5:
            raise ValueError("Need at least 5 valid chessboard images for calibration")

        ret, self.camera_matrix, self.dist_coeffs, self.rvecs, self.tvecs = (
            cv2.calibrateCamera(self.objpoints, self.imgpoints, img_size, None, None)
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

        return self.error

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
    parser = argparse.ArgumentParser(description="Camera Calibration Script")
    parser.add_argument("--device", type=int, default=0, help="Camera device number")
    parser.add_argument(
        "--output",
        type=str,
        default="camera_calibration.npz",
        help="Output file for calibration data",
    )
    parser.add_argument(
        "--squares-x",
        type=int,
        default=9,
        help="Number of inner corners on x axis of the chessboard",
    )
    parser.add_argument(
        "--squares-y",
        type=int,
        default=6,
        help="Number of inner corners on y axis of the chessboard",
    )
    parser.add_argument(
        "--square-size",
        type=float,
        default=0.025,
        help="Size of a chessboard square in meters",
    )
    args = parser.parse_args()

    cap = cv2.VideoCapture(args.device)
    calibrator = CameraCalibrator(
        chessboard_size=(args.squares_x, args.squares_y), square_size=args.square_size
    )

    img_counter = 0

    print("Press 'c' to capture an image for calibration")
    print("Press 'ESC' to finish and compute calibration")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        ret, drawn_frame = calibrator.find_chessboard_corners(frame.copy())

        key = cv2.waitKey(1)
        if key == 27:  # ESC
            break
        elif key == ord("c"):
            if ret:
                img_counter += 1
                print(
                    f"Image {img_counter} captured - {len(calibrator.objpoints)} valid chessboards found"
                )
            else:
                print("No chessboard detected in current frame")

    cap.release()

    if len(calibrator.objpoints) > 0:
        img_size = (frame.shape[1], frame.shape[0])

        error = calibrator.calibrate(img_size)

        calibrator.save_calibration(args.output)
    else:
        print("No valid calibration data collected")


if __name__ == "__main__":
    main()
