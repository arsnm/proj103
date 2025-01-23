import json
import requests
import time
import threading
from src.motor.motor_controller import MotorController
from src.vision.vision_controller import VisionController
from models.message import Message, MessageType
from models.race import TeamStatus, MarkerStatus, RaceStatus
from src.models.race import RaceStatus
from src.utils.grid_navigation import match_coord_to_case


class TrackingServerManager:
    """Manages hhtp communication with tracking server"""

    def __init__(
        self,
        url: str,
        race_controller=None,
        motor_controller=None,
        vision_controller=None,
        test_mode=False,
    ):
        self.url = url
        self.race_controller = race_controller
        self.motor_controller = motor_controller
        self.vision_controller = vision_controller
        self.http_session = None
        self.running = False
        self.connected = False
        self.team_id: int = 5  # TrackingServerConfig.TEAM_ID
        self.race_status = None
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

    # def send_marker(self, marker_id, marker_position):
    #     if self.test_mode:
    #         print("TEST MODE - Sent marker to tracking server")
    #         return
    #     id = self.team_id
    #     x, y = marker_position
    #     row, col = match_coord_to_case(x, y)
    #     url = f"{self.url}/marker"
    #     params_list = [{"id": id, "col": col, "row": row}]
    #     for params in params_list:
    #         try:
    #             async with self.http_session.post(url, params=params) as response:
    #                 message = TrackingServerConfig.API_RESPONSE.get(
    #                     response.status, "Response from server not recognized"
    #                 )
    #                 print(f"Marker sent: {message}")
    #         except Exception as e:
    #             print(f"ERROR - Tracking server error : {e}")
    #
    # async def update_race_status(self):
    #     if self.test_mode:
    #         print("TEST MODE - Updating race status from server")
    #         return
    #     url = f"{self.url}/status"
    #     try:
    #         async with self.http_session.get(url) as response:
    #             message = TrackingServerConfig.API_RESPONSE.get(
    #                 response.status, "Response from server not recognized"
    #             )
    #             if response.status in [200, 503]:
    #                 data = await response.text()
    #                 data = json.loads(data)
    #                 self.race_status = RaceStatus(data)
    #             else:
    #                 print("ERROR - Could not receive race status from tracking server")
    #     except:
    #         return
    #

    def stop(self):
        """Stop HTTP periodic communication"""
        self.running = False


def main(
    server_url: str,
    tracking_type: str,
    pos_x: float,
    pos_y: float,
    marker_id: Optional[int],
):
    manager = TrackingServerManager("http://proj103.r2.enst.fr/api")
    if tracking_type == "pos":
        asyncio.run(manager.update_position((pos_x, pos_y)))
    elif tracking_type == "marker":
        asyncio.run(manager.send_marker())
