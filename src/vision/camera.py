import cv2
import numpy as np
import os
from typing import Tuple, Optional
from pathlib import Path
import sys

sys.path.append(
    str(Path(__file__).parent.parent.parent)
)  # Go up to repo root, then src will be in path

from src.config import VisionConfig


class CameraManager:
    def __init__(self, config: VisionConfig, test_mode: bool = False):
        self.config = config
        self.camera = None
        self.test_mode = test_mode

    def initialize(self):
        """Initialize camera capture"""
        if not self.test_mode:
            self.camera = cv2.VideoCapture(0)
            # Set camera properties if needed
            self.camera.set(cv2.CAP_PROP_FPS, self.config.CAMERA_FPS)
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    def read_frame(self) -> Tuple[bool, Optional[cv2.Mat]]:
        """Read a frame from camera"""
        if self.test_mode:
            # Create a test frame with a simple pattern
            frame = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(
                frame,
                "TEST MODE",
                (50, 50),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (255, 255, 255),
                2,
            )
            return True, frame

        if self.camera is None:
            return False, None
        return self.camera.read()

    def release(self):
        """Release camera resources"""
        if self.camera is not None:
            self.camera.release()
            self.camera = None

    def take_picture(
        self, directory: str = "pictures", filename: Optional[str] = None
    ) -> bool:
        """
        Capture a single frame and save it to the specified directory.

        Args:
            directory: Directory to save the picture (default: 'pictures')
            filename: Optional filename (default: timestamp)

        Returns:
            bool: True if successful, False otherwise
        """
        # Create directory if it doesn't exist
        if not os.path.exists(directory):
            os.makedirs(directory)

        # Read frame
        success, frame = self.read_frame()
        if not success:
            return False

        # Generate filename if not provided
        if filename is None:
            from datetime import datetime

            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"capture_{timestamp}.jpg"

        # Save frame
        filepath = os.path.join(directory, filename)
        return cv2.imwrite(filepath, frame)


def main():
    """
    Main function for CLI usage.
    Accepts single character input:
    - 'c': Capture a picture with a 3-second delay
    - 'q': Quit the program
    """
    from datetime import datetime
    import time

    # Initialize camera
    config = VisionConfig()  # You'll need to import or define this
    camera = CameraManager(config)
    camera.initialize()

    print("Enter 'c' to capture a picture (3-second delay)")
    print("Enter 'q' to quit")

    try:
        while True:
            char = input("Enter a command: ").strip().lower()  # Take user input

            if char == "c":
                print("Picture will be taken in 3 seconds...")
                time.sleep(3)  # Wait for 3 seconds before capturing
                success = camera.take_picture()
                if success:
                    print(f"Picture taken at {datetime.now().strftime('%H:%M:%S')}")
                else:
                    print("Failed to take picture")
            elif char == "q":
                print("Exiting the program...")
                break
            else:
                print("Invalid command. Enter 'c' to capture or 'q' to quit.")

    finally:
        # Cleanup
        camera.release()
        print("Camera resources released.")


if __name__ == "__main__":
    main()
