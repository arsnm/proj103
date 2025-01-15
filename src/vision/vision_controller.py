import cv2
import numpy as np
import threading
import subprocess
from src.config import NetworkConfig, ArucoConfig, PositionConfig
from src.utils.grid_navigation import (
    relative_to_absolute_coord,
    id_to_orientation,
    orientation_to_quadrant,
)
from src.vision import aruco_detector
from src.vision.camera_controller import CameraController
from src.vision.aruco_detector import pose_estimation_solve_pnp


class VisionController:

    def __init__(self, camera_controller, odometry_controller):
        self.camera = camera_controller
        self.odometry_controller = odometry_controller
        self.position = (0, 0, 0)
        self.last_frame = None
        self.rtmp_url = NetworkConfig.RMTP_URL
        self.running = False
        self.vision_thread = None

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
            "./static/stream.m3u8",  # Output HLS files to the static directory
        ]

        process = subprocess.Popen(ffmpeg_cmd, stdin=subprocess.PIPE)

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
                frame, ArucoConfig.ARUCO_DICT, self.camera.mtx, self.camera.dist
            )

            self.flag_detection(output["markers"])

            self.update_position(output["markers"])

            process.stdin.write(output["frame"].tobytes())

        # Cleanup
        process.stdin.close()
        process.wait()
        self.camera.release_camera()

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join()

    def handle_error_odometry_vision(self):
        odo_x, odo_y, odo_orientation = self.odometry_controller.get_position()
        if abs(odo_x - self.position[0]) > PositionConfig.X_DIFF_THRESHOLD:
            odo_x = (odo_x + self.position[0]) / 2
        if abs(odo_y - self.position[1]) > PositionConfig.Y_DIFF_THRESHOLD:
            odo_y = (odo_y + self.position[1]) / 2
        if (
            abs(odo_orientation - self.position[2])
            > PositionConfig.ORIENTATION_DIFF_THRESHOLD
        ):
            odo_orientation = (odo_orientation + self.position[2]) / 2
        self.odometry_controller.update_position(odo_x, odo_y, odo_orientation)
        self.position = (odo_x, odo_y, odo_orientation)

    def update_position(self, list_marker):
        x, y, orientation = 0, 0, 0
        count = 0
        if len(list_marker) > 0:
            for marker in list_marker:
                id, x_rel, y_rel, angle = marker
                marker_orientation = id_to_orientation(id)
                orientation += (marker_orientation + angle) % (2 * np.pi)
                quadrant = orientation_to_quadrant(orientation)
                x_temp, y_temp = relative_to_absolute_coord(id, quadrant, x_rel, y_rel)
                if x_temp and y_temp:
                    x += x_temp
                    y += y_temp
                    count += 1
                else:
                    orientation -= (marker_orientation + angle) % (2 * np.pi)
        self.position = x / count, y / count, orientation / count

        # self.handle_error_odometry_vision()

    def flag_detection(self, list_marker):
        flag_detected = []

    def get_position(self):
        return self.position
