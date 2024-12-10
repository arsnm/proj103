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
    Press 'c' to capture a picture
    Press Ctrl+C to exit
    """
    import signal
    from datetime import datetime

    # Initialize camera
    config = VisionConfig()  # You'll need to import or define this
    camera = CameraManager(config)
    camera.initialize()

    # Flag for graceful shutdown
    running = True

    def signal_handler(sig, frame):
        nonlocal running
        print("\nShutting down...")
        running = False

    # Register Ctrl+C handler
    signal.signal(signal.SIGINT, signal_handler)

    print("Press 'c' to capture a picture")
    print("Press Ctrl+C to exit")

    try:
        while running:
            # Check for 'c' key press (waitKey returns -1 if no key was pressed)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("c"):
                success = camera.take_picture()
                if success:
                    print(f"Picture taken at {datetime.now().strftime('%H:%M:%S')}")
                else:
                    print("Failed to take picture")

    finally:
        # Cleanup
        camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
