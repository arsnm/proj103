import cv2
import os
import time
import argparse
import platform
from datetime import datetime
from src.config import CameraConfig


class CameraController:
    def __init__(self, camera_index=0, save_directory="pictures"):
        """
        Initialize the camera controller.

        Parameters:
        - camera_index (int): Index of the camera (default is 0).
        - save_directory (str): Directory where images will be saved (default is "pictures").
        """
        self.camera_index = camera_index
        self.save_directory = save_directory
        self.cap = cv2.VideoCapture(self.camera_index)

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, CameraConfig.WIDTH.value)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CameraConfig.HEIGHT.value)
        self.cap.set(cv2.CAP_PROP_FPS, CameraConfig.FPS.value)

        if not os.path.exists(self.save_directory):
            os.makedirs(self.save_directory)

    def capture_image(self):
        """
        Capture an image from the camera and save it to the specified directory.
        The image file will be named with the current date and time.
        """
        ret, frame = self.cap.read()
        if ret:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = os.path.join(self.save_directory, f"image_{timestamp}.png")
            cv2.imwrite(filename, frame)
            print(f"Image saved to {filename}")
        else:
            print("Failed to capture image.")

    def read(self):
        return self.cap.read()

    def release_camera(self):
        self.cap.release()
        cv2.destroyAllWindows()

    def __del__(self):
        self.release_camera()


def is_raspberry_pi():
    """
    Check if the current OS is Raspberry Pi OS.
    """
    return platform.system() == "Linux" and "rpt-rpi" in platform.uname().version


if __name__ == "__main__":
    # Argument parser setup
    parser = argparse.ArgumentParser(description="Camera Controller Application")
    parser.add_argument(
        "-d", "--dir", type=str, default="pictures", help="Directory to save images"
    )
    parser.add_argument(
        "-i", "--index", type=int, default=0, help="Camera index to use"
    )
    args = parser.parse_args()

    # Create the camera controller with provided arguments
    camera_controller = CameraController(
        camera_index=args.index, save_directory=args.dir
    )
    print("Press 'c' to capture an image or 'q' to quit.")
    print(f"Images will be saved in {args.dir}")

    while True:
        ret, frame = camera_controller.cap.read()
        if not ret:
            print("Failed to read from camera.")
            break

        if not is_raspberry_pi():
            cv2.imshow("Camera", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("c"):
            print("Taking picture in 2s...")
            time.sleep(2)
            camera_controller.capture_image()
        elif key == ord("q"):
            print("Quitting...")
            break

    camera_controller.release_camera()
