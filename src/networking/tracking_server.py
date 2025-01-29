import json
import requests
import time
import threading
from typing import Optional
from src.motor.motor_controller import MotorController
from src.vision.vision_controller import VisionController
from src.models.race import RaceStatus
from src.utils.grid_navigation import match_coord_to_case
from src.config import NetworkConfig


class TrackingServerManager:
    """Manages hhtp communication with tracking server"""

    def __init__(
        self,
        url: str,
        motor_controller=None,
        vision_controller=None,
        race_controller=None,
        test_mode=False,
    ):
        self.url = url
        self.motor_controller: Optional[MotorController] = motor_controller
        self.vision_controller: Optional[VisionController] = vision_controller
        self.http_session = None
        self.running = False
        self.connected = False
        self.team_id: int = NetworkConfig.TRACKING_SERVER_TEAM_ID.value
        self.race_status: Optional[RaceStatus] = None
        self.test_mode = test_mode

    def start(self):
        if not self.running:
            self.running = True
            self.thread = threading.Thread(target=self._update_loop, daemon=True)
            self.thread.start()

    def _update_loop(self):
        while self.running:
            try:
                if self.motor_controller:
                    position = self.motor_controller.get_position()
                elif self.vision_controller:
                    position = self.vision_controller.get_position()
                else:
                    position = None

                self.send_position(position)

                # self.update_race_status()

                time.sleep(1)
            except Exception as e:
                print(f"ERROR : Periodic send to tracking server failed - {e}")
                time.sleep(1)

    def send_position(self, new_position=None):
        """Send the given position to the tracking server"""
        if self.test_mode:
            print("TEST MODE - Sent position to tracking server")
            return
        if new_position is None:
            print("ERROR - No valid position to send to tracking server")
            return
        x, y, _ = new_position  # a position always contains orientation
        x, y = int(x * 100), int(y * 100)
        url = f"{self.url}/pos"
        try:
            response = requests.post(f"{url}?x={x}&y={y}")
            print(f"Position updated (supposedly)")
        except Exception as e:
            print(f"ERROR - Tracking server error: {e}")

    def send_marker(self, marker_id, marker_position, scan=False):
        if self.test_mode:
            print("TEST MODE - Sent marker to tracking server")
            return
        x, y, _ = marker_position
        case = match_coord_to_case(x, y)
        if case is None:
            print("ERROR - Coord provided for marker are not withing the grid range")
            return
        row, col = case
        url = f"{self.url}/marker"
        try:
            if scan:
                response = requests.post(
                    f"{url}?id={marker_id}&col{col}&row={row}&scan=false"
                )
            else:
                response = requests.post(
                    f"{url}?id={marker_id}&col{col}&row={row}&scan=true"
                )
            print(f"Position updated (supposedly)")
        except Exception as e:
            print(f"ERROR - Tracking server error: {e}")

    def update_race_status(self):
        if self.test_mode:
            print("TEST MODE - Updating race status from server")
            return

        # URL to send the GET request to
        url = f"{self.url}/status"

        # Sending the GET request
        response = requests.get(url)

        # Checking if the request was successful
        if response.status_code == 200:
            data = response.json()
            self.race_status = RaceStatus(data)

        elif response.status_code == 503:
            print("Getting status should have worked, but race isnt'started.")

    def stop(self):
        """Stop HTTP periodic communication"""
        self.running = False
