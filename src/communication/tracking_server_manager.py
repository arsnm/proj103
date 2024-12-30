import json
import aiohttp
import asyncio
from typing import Dict, Callable, Optional, Tuple
from models.message import Message, MessageType
from models.race import TeamStatus, MarkerStatus, RaceStatus
from config import TrackingServerConfig
from controllers.race_controller import RaceController


class TrackingServerManager:
    """Manages hhtp communication with tracking server"""

    def __init__(
        self,
        url: str,
        race_controller: Optional[RaceController] = None,
        test_mode=False,
    ):
        self.url = url
        self.http_session: Optional[aiohttp.ClientSession] = None
        self.running = True
        self.connected = False
        self.team_id: int = TrackingServerConfig.TEAM_ID
        self.race_controller = race_controller
        self.TEST_MODE = test_mode

    def connect(self):
        """Establish HTTP connection to tracking server"""
        if self.running and not self.connected:
            self.http_session = aiohttp.ClientSession()
            self.connected = True

    async def update_position(self, new_pose: Optional[Tuple[float, float]]):
        """Send the given position to the tracking server"""
        if self.TEST_MODE:
            print("[TEST_MODE] - Sent position to tracking server")
            return
        if new_pose is None:
            print("ERROR - No valid position to send to tracking server")
            return
        x, y = new_pose
        params_list = [{"x": int(x), "y": int(y)}]
        url = f"{self.url}/pos"
        for params in params_list:
            try:
                async with self.http_session.post(url, params=params) as response:
                    message = TrackingServerConfig.API_RESPONSE.get(
                        response.status, "Response from server not recognized"
                    )
                    print(f"Position update : {message}")
            except Exception as e:
                print(f"ERROR - Tracking server error: {e}")

    async def send_marker(self, marker_id: int, marker_pos: Tuple[float, float]):
        if self.TEST_MODE:
            print("[TEST_MODE] - Sent marker to tracking server")
            return
        id = self.team_id
        x, y = marker_pos
        row, col = match_coord_to_case(x, y)
        url = f"{self.url}/marker"
        params_list = [{"id": id, "col": col, "row": row}]
        for params in params_list:
            try:
                async with self.http_session.post(url, params=params) as response:
                    message = TrackingServerConfig.API_RESPONSE.get(
                        response.status, "Response from server not recognized"
                    )
                    print(f"Marker sent: {message}")
            except Exception as e:
                print(f"ERROR - Tracking server error : {e}")

    async def update_race_status(self):
        if self.TEST_MODE:
            print("[TEST_MODE] - Updating race status from server")
            return
        url = f"{self.url}/status"
        try:
            async with self.http_session.get(url) as response:
                message = TrackingServerConfig.API_RESPONSE.get(
                    response.status, "Response from server not recognized"
                )
                if response.status in [200, 503]:
                    data = await response.text()
                    data = json.loads(data)
                    self.race_controller.update_status(data)
                else:
                    print("ERROR - Could not receive race status from tracking server")
        except:
            return

    async def stop(self):
        """Stop HTTP communication"""
        self.running = False
        if self.http_session:
            await self.http_session.close()


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
