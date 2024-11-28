from typing import Optional
from dataclasses import dataclass
import time


@dataclass
class Position:
    """Represents a complete robot position and orientation in 2D space"""

    x: float
    y: float
    theta: float  # Robot orientation in world frame
    timestamp: Optional[float] = None
    camera_angle: Optional[float] = None  # Camera orientation relative to robot
    confidence: float = 1.0  # Detection confidence (0-1)

    def __post_init__(self):
        if self.timestamp is None:
            self.timestamp = time.time()
