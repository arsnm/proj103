from models.race import TeamStatus, MarkerStatus, RaceStatus
from typing import Optional, Dict, List, Tuple
from time import time_ns


class RaceController:
    """Race controller that keeps track of the race status and updates"""

    def __init__(self, team_id: int = 15, flags_range: range = range(0, 0)):
        self.race_status = RaceStatus()
        self.flags_range = flags_range
        self.team_id = team_id

    def new_flags(self, found_flags: List[int], flags_pos):
        """Return the new flags to be sent, when flags are found"""
        new_flag_ids = []
        new_flag_poses = []
        for flag, pose in zip(found_flags, flags_pos):
            if flag not in self.race_status.markers and flag in self.flags_range:
                new_flag_ids.append(flag)
                new_flag_poses.append(pose)
        return new_flag_ids, new_flag_poses

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
        self.race_status.last_updated = time_ns()

    def get_status(self):
        return self.race_status.get_status()

    def update_register(self, register_id: int, data: Dict):
        pass
