from models.race import TeamStatus, MarkerStatus, RaceStatus
from typing import Optional, Dict, List, Tuple
import time


class RaceController:
    """Race controller that keeps track of the race status and updates"""

    def __init__(self, team_id: int = 15, flags_range: Tuple[int, int] = (5, 17)):
        self.race_status = RaceStatus()
        self.flags_range = flags_range  # ranged [| a , b [|, where a < b (else empty)
        self.team_id = team_id

    def new_flags(self, found_flags: List[int]):
        """Return the new flags to be sent, when flags are found"""
        new_flags = []
        for flag in found_flags:
            if flag not in self.race_status.markers and flag in range(
                *self.flags_range
            ):
                new_flags.append(flag)
        return new_flags

    def update_status(self, race_status: Dict):
        """Update the race status with data received from server"""
        self.race_status.status = race_status["status"]
        self.race_status.elapsed = race_status["elapsed"]
        for position in race_status["positions"]:
            team_id = position["team"]
            self.race_status.positions[team_id] = TeamStatus(position)
        for marker in race_status["markers"]:
            id = marker["id"]
            self.race_status.markers[id] = MarkerStatus(marker)
        self.race_status.last_update = time_ns()
