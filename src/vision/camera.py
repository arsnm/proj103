import cv2
from typing import Tuple, Optional
from config import VisionConfig


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
