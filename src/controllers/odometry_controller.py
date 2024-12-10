import numpy as np
import time
from models.position import Position
from config import ControlConfig


class OdometryTracker:
    def __init__(self, config: ControlConfig):
        self.config = config
        self.wheel_base = config.WHEEL_BASE  # cm
        self.ticks_per_rotation = config.TICKS_PER_ROTATION
        self.wheel_circumference = 2 * np.pi * (config.WHEEL_RADIUS)  # cm
        self.cm_per_tick = self.wheel_circumference / self.ticks_per_rotation

        # Current position estimate in cm
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # radians

        # Previous encoder values in ticks
        self.last_left_ticks = 0
        self.last_right_ticks = 0
        self.last_update = time.time()

        # Error detection parameters
        self.max_ticks_per_update = int(self.ticks_per_rotation / 4)
        self.position_confidence = 1.0
        self.min_confidence = 0.2

    def update_position(self, left_ticks: int, right_ticks: int) -> Position:
        """Update position based on encoder ticks"""
        current_time = time.time()
        dt = current_time - self.last_update

        if dt <= 0:
            return Position(
                x=float(self.x),
                y=float(self.y),
                theta=float(self.theta),
                camera_angle=0.0,
                timestamp=current_time,
                confidence=self.position_confidence,
            )

        # Calculate tick differences
        delta_left = left_ticks - self.last_left_ticks
        delta_right = right_ticks - self.last_right_ticks

        # Detect potential encoder errors
        if (
            abs(delta_left) > self.max_ticks_per_update
            or abs(delta_right) > self.max_ticks_per_update
        ):
            self.position_confidence = max(
                self.position_confidence * 0.8, self.min_confidence
            )
        else:
            self.position_confidence = min(self.position_confidence * 1.1, 1.0)

        # Convert ticks to distances in cm
        left_distance = delta_left * self.cm_per_tick
        right_distance = delta_right * self.cm_per_tick

        # Calculate robot movement
        distance = (left_distance + right_distance) / 2
        rotation = (right_distance - left_distance) / self.wheel_base

        # Update position using differential drive kinematics
        if abs(rotation) < 1e-6:
            # Straight line motion
            dx = distance * np.cos(self.theta)
            dy = distance * np.sin(self.theta)
            self.x += dx
            self.y += dy
        else:
            # Arc motion
            radius = distance / rotation
            dx = radius * (np.sin(self.theta + rotation) - np.sin(self.theta))
            dy = radius * (-np.cos(self.theta + rotation) + np.cos(self.theta))
            self.x += dx
            self.y += dy
            self.theta += rotation

        # Normalize theta to [-pi, pi]
        self.theta = np.arctan2(np.sin(self.theta), np.cos(self.theta))

        # Update stored values
        self.last_left_ticks = left_ticks
        self.last_right_ticks = right_ticks
        self.last_update = current_time

        return Position(
            x=float(self.x),
            y=float(self.y),
            theta=float(self.theta),
            camera_angle=0.0,
            timestamp=current_time,
            confidence=self.position_confidence,
        )

    def reset_position(
        self, position: Position, left_ticks: int, right_ticks: int
    ) -> None:
        """Reset odometry to match a known position"""
        self.x = position.x
        self.y = position.y
        self.theta = position.theta
        self.last_left_ticks = left_ticks
        self.last_right_ticks = right_ticks
        self.position_confidence = position.confidence
        self.last_update = time.time()
