import cv2
import numpy as np
import threading
import subprocess
import time
from src.config import ArucoConfig, PositionConfig
from src.utils.grid_navigation import (
    relative_to_absolute_coord,
    id_to_orientation,
    orientation_to_quadrant,
)
from src.vision.camera_controller import CameraController
from src.vision.aruco_detector import pose_estimation_solve_pnp
from src.motor.motor_controller import MotorController
from src.motor.odometry_controller import OdometryController
from src.control.automatic import AutomaticController
from src.control.target import TargetController


class VisionController:

    def __init__(
        self,
        camera_controller: CameraController,
    ):
        self.camera = camera_controller
        self.position = (0, 0, 0)
        self.last_pos_update = -1
        self.last_frame = None
        self.running = False
        self.vision_thread = None
        self.flag_detected = {}
        self.flag_detected_matrix = {}

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._process_stream)
        self.thread.start()

    def _process_stream(self):
        width = int(self.camera.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(self.camera.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = int(self.camera.cap.get(cv2.CAP_PROP_FPS))

        ffmpeg_cmd = [
            "ffmpeg",
            "-y",
            "-f",
            "rawvideo",
            "-vcodec",
            "rawvideo",
            "-pix_fmt",
            "bgr24",
            "-s",
            f"{width}x{height}",
            "-r",
            str(fps),
            "-i",
            "-",
            "-c:v",
            "libx264",
            "-preset",
            "ultrafast",
            "-f",
            "hls",
            "-hls_time",
            "1",
            "-hls_list_size",
            "3",
            "-hls_flags",
            "delete_segments",
            "./src/web/static/stream.m3u8",  # Output HLS files to the static directory
        ]

        # process = subprocess.Popen(ffmpeg_cmd, stdin=subprocess.PIPE)

        while self.running:
            ret, frame = self.camera.read()
            if not ret:
                print("ERROR - Could not read frame.")
                if self.last_frame:
                    frame = self.last_frame
                else:
                    break
            else:
                self.last_frame = frame
            output = pose_estimation_solve_pnp(
                frame, ArucoConfig.ARUCO_DICT.value, self.camera.mtx, self.camera.dist
            )

            self.update_position(output["markers"])

            self.flag_detection(output["markers"], output["matrix"])

            # process.stdin.write(output["frame"].tobytes())

        # # Cleanup
        # process.stdin.close()
        # process.wait()
        # self.camera.release_camera()

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join()
            self.camera.release_camera()

    def update_position(self, list_marker):
        x, y, orientation = 0, 0, 0
        count = 0
        if len(list_marker) > 0:
            for marker in list_marker:
                id, x_rel, y_rel, angle = marker
                if id in range(1, 5):
                    marker_orientation = id_to_orientation(id)
                    orientation += (marker_orientation + angle) % (2 * np.pi)
                    quadrant = orientation_to_quadrant(orientation)
                    x_temp, y_temp = relative_to_absolute_coord(
                        id, quadrant, x_rel, y_rel
                    )
                    if x_temp and y_temp:
                        x += x_temp
                        y += y_temp
                        count += 1
                    else:
                        orientation -= (marker_orientation + angle) % (2 * np.pi)
        if count != 0:
            self.position = x / count, y / count, orientation / count
            self.last_pos_update = time.time()

    def flag_detection(self, list_marker, list_matrix):
        for marker in list_marker:
            id, x_rel, y_rel, angle = marker
            if (
                np.sqrt(x_rel**2 + y_rel**2) < ArucoConfig.FLAG_DIST_THRESHOLD.value
                and abs(angle) < ArucoConfig.FLAG_ANGLE_THRESHOLD.value
            ):
                if id == 0:
                    # self.automatic_controller.notify_hint()
                    pass

                elif id in range(5, 50) and id not in self.flag_detected:
                    self.flag_detected[id] = {
                        "x": self.position[0],
                        "y": self.position[1],
                    }
            for marker in list_matrix:
                id, tvec, rvec = marker
                if id == 0 or (id in range(5, 50) and id not in self.flag_detected):
                    self.flag_detected_matrix[id] = (id, tvec, rvec)

    def get_flags_matrix(self):
        return self.flag_detected_matrix

    def get_flags(self):
        return self.flag_detected

    def get_position(self):
        return self.position
