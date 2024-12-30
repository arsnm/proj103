from typing import Optional, Dict, List
from dataclasses import dataclass
import time
from src.config import TrackingServerConfig


# GET status request example
example = '[{"status":2, "elapsed":0, "positions":[{"team":1,"x":100,"y":20,"num_markers":1}],"markers":[{"team": 1, "id": 6, "col": 1, "row": "B", "time": 2000, "valid": true}]}'


class TeamStatus:
    """Represents a complete racing-team status"""

    def __init__(self, team_status: Dict[str, int]):
        self.team: int = team_status["team"]
        self.x: int = team_status["x"]  # in cm
        self.y: int = team_status["y"]  # in cm
        self.num_markers: int = team_status[
            "num_markers"
        ]  # number of markers captured by the team


class MarkerStatus:
    """Represents a complete marker status"""

    def __init__(self, marker_status, sent=True):
        self.team: int = marker_status["team"]  # team who found the marker
        self.id: int = marker_status["id"]
        self.col: int = marker_status[
            "col"
        ]  # x-coordinate of the case where the marker was found (1, 2, ...)
        self.row: str = marker_status[
            "row"
        ]  # y-coordinate of the case where the marker was found (A, B, ...)
        self.time: int = marker_status["time"]  # in ms
        self.valid: bool = marker_status["valid"]
        self.sent: bool = sent
        self.scan: bool = marker_status["scan"]


class RaceStatus:
    """Represents a complete race status"""

    def __init__(self, race_status: Optional[Dict] = None):
        if race_status:
            self.status: int = race_status["status"]  # status of the race
            self.elapsed: int = race_status[
                "elapsed"
            ]  # time elpased since start (in ms)
            self.positions: Dict[int, TeamStatus] = {}
            for team_status in race_status["positions"]:
                team_id = team_status["team"]
                self.positions[team_id] = TeamStatus(team_status)
            self.markers: Dict[int, MarkerStatus] = {}
            for marker_status in race_status["markers"]:
                marker_id = marker_status["id"]
            self.initialized = True
        else:
            self.initialized = False
        self.last_updated: Optional[int] = None

    def get_positions(self) -> Optional[List[Dict]]:
        if self.initialized:
            positions = []
            for team_id in self.positions:
                positions.append(
                    {
                        "team": team_id,
                        "x": self.positions[team_id].x,
                        "y": self.positions[team_id].y,
                        "num_markers": self.positions[team_id].num_markers,
                    }
                )
            return positions

    def get_markers(self) -> Optional[List[Dict]]:
        if self.initialized:
            markers = []
            for marker_id in self.markers:
                marker = self.markers[marker_id]
                markers.append(
                    {
                        "team": marker.team,
                        "id": marker.id,
                        "col": marker.col,
                        "row": marker.row,
                        "time": marker.time,
                        "valid": marker.valid,
                        "scan": marker.scan,
                    }
                )

    def get_status(self) -> Optional[Dict]:
        if self.initialized:
            return {
                "status": self.status,
                "elapsed": self.elapsed,
                "positions": self.get_positions(),
                "markers": self.get_markers(),
            }
