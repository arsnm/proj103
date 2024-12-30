from dataclasses import dataclass
from enum import Enum
import numpy as np
from typing import Tuple, Callable
from src.utils.conversion_utils import *
from src.config import ControlConfig
import queue


@dataclass
class PositionConfig:
    # Grid parameters
    GRID_CASE_SIZE_CM: float = ControlConfig.GRID_CASE_SIZE

    # Robot parameters
    WHEEL_RADIUS_CM: float = ControlConfig.WHEEL_RADIUS_CM
    WHEEL_BASE_CM: float = ControlConfig.WHEEL_BASE_CM
    TICKS_PER_ROTATION: int = ControlConfig.TICKS_PER_ROTATION

    # Control parameters
    POSITION_THRESHOLD_CM: float = ControlConfig.POSITION_THRESHOLD_CM
    ANGLE_THRESHOLD_RAD: float = ControlConfig.ANGLE_THRESHOLD_RAD
    MAX_SPEED_TICKS: int = ControlConfig.MAX_SPEED_TICKS

    @property
    def CM_PER_TICK(self) -> float:
        wheel_circumference = 2 * np.pi * self.WHEEL_RADIUS_CM
        return wheel_circumference / self.TICKS_PER_ROTATION


class MovementState(Enum):
    MOVING = 1
    TURNING = 2

    QUEUED = 3

    DONE = 4


class PositionController:
    def __init__(self, config: PositionConfig):
        self.config = config

        # Current position state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Target state
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_theta = 0.0

        # Start state
        self.start_x = 0.0
        self.start_y = 0.0
        self.start_theta = 0.0

        self.state = MovementState.DONE

        self.movement_queue = queue.Queue()

    def move_distance(self, distance_cm: float) -> None:
        """Move forward (positive) or backward (negative) by a specified distance in cm"""
        self.target_distance_cm = self.x + distance_cm
        self.start_x_cm = self.x_cm
        self.start_y_cm = self.y_cm
        self.state = MovementState.MOVING_DISTANCE

    def turn_angle(self, angle_rad: float) -> None:
        """Turn by a specified angle in radians (-π to π)"""
        # Normalize angle to [-pi, pi]
        angle_rad = np.arctan2(np.sin(angle_rad), np.cos(angle_rad))
        self.target_theta_rad = self.theta_rad + angle_rad
        self.state = MovementState.TURNING_ANGLE

    def move_to_case(self, target_case: str, final_theta_rad: float) -> None:
        """Start movement to target case with final orientation"""
        self.target_x_cm, self.target_y_cm = match_case_to_coord(target_case)
        self.target_theta_rad = final_theta_rad
        self.state = MovementState.ALIGNING_Y

    def update_position(self, delta_left_ticks: int, delta_right_ticks: int) -> None:
        """Update position based on encoder tick changes"""
        # Convert ticks to distances
        left_dist = delta_left_ticks * self.config.CM_PER_TICK
        right_dist = delta_right_ticks * self.config.CM_PER_TICK

        # Calculate robot movement
        center_dist = (left_dist + right_dist) / 2
        delta_theta = (right_dist - left_dist) / self.config.WHEEL_BASE_CM

        # Update position
        self.x_cm += center_dist * np.cos(self.theta_rad)
        self.y_cm += center_dist * np.sin(self.theta_rad)
        self.theta_rad += delta_theta

        # Normalize angle to [-pi, pi]
        self.theta_rad = np.arctan2(np.sin(self.theta_rad), np.cos(self.theta_rad))

    def update_trusted_position(self, x: float, y: float, theta_rad: float) -> None:
        """Update position from trusted external source"""
        self.x = x
        self.y = y
        self.theta_rad = theta_rad

    def compute_wheel_speeds(self) -> Tuple[int, int]:
        """Compute required wheel speeds based on current state and position"""
        if self.state == MovementState.DONE:
            return 0, 0

        speed = self.config.MAX_SPEED_TICKS

        if self.state == MovementState.MOVING_DISTANCE:
            # Calculate remaining distance
            distance_moved = self.get_distance_moved()
            remaining_distance = self.target_distance_cm - distance_moved

            if abs(remaining_distance) > self.config.POSITION_THRESHOLD_CM:
                # Add angle correction to maintain straight line
                initial_angle = self.theta_rad - (
                    np.pi if self.target_distance_cm < 0 else 0
                )
                angle_error = initial_angle - self.theta_rad
                angle_error = np.arctan2(np.sin(angle_error), np.cos(angle_error))
                correction = int(angle_error * speed * 0.2)  # 20% max correction

                # Set direction based on target distance sign
                direction = np.sign(self.target_distance_cm)
                base_speed = speed * direction
                return (base_speed - correction, base_speed + correction)
            else:
                self.state = MovementState.DONE
                return 0, 0

        elif self.state == MovementState.TURNING_ANGLE:
            angle_error = self.target_theta_rad - self.theta_rad
            angle_error = np.arctan2(np.sin(angle_error), np.cos(angle_error))

            if abs(angle_error) > self.config.ANGLE_THRESHOLD_RAD:
                return (
                    -speed if angle_error > 0 else speed,
                    speed if angle_error > 0 else -speed,
                )
            else:
                self.state = MovementState.DONE
                return 0, 0

        if self.state in [
            MovementState.ALIGNING_Y,
            MovementState.ALIGNING_X,
            MovementState.FINAL_ROTATION,
        ]:
            # Handle rotation states
            if self.state == MovementState.ALIGNING_Y:
                target_angle = 0 if self.target_y_cm > self.y_cm else np.pi
            elif self.state == MovementState.ALIGNING_X:
                target_angle = np.pi / 2 if self.target_x_cm > self.x_cm else -np.pi / 2
            else:  # FINAL_ROTATION
                target_angle = self.target_theta_rad

            angle_error = target_angle - self.theta_rad
            angle_error = np.arctan2(np.sin(angle_error), np.cos(angle_error))

            if abs(angle_error) > self.config.ANGLE_THRESHOLD_RAD:
                return (
                    -speed if angle_error > 0 else speed,
                    speed if angle_error > 0 else -speed,
                )
            else:
                self.state = {
                    MovementState.ALIGNING_Y: MovementState.MOVING_Y,
                    MovementState.ALIGNING_X: MovementState.MOVING_X,
                    MovementState.FINAL_ROTATION: MovementState.DONE,
                }[self.state]
                return 0, 0

        elif self.state in [MovementState.MOVING_Y, MovementState.MOVING_X]:
            # Handle movement states
            if self.state == MovementState.MOVING_Y:
                error = self.target_y_cm - self.y_cm
                current_angle = 0 if error > 0 else np.pi
            else:  # MOVING_X
                error = self.target_x_cm - self.x_cm
                current_angle = np.pi / 2 if error > 0 else -np.pi / 2

            if abs(error) > self.config.POSITION_THRESHOLD_CM:
                # Add small angle correction to maintain straight line
                angle_error = current_angle - self.theta_rad
                angle_error = np.arctan2(np.sin(angle_error), np.cos(angle_error))
                correction = int(angle_error * speed * 0.2)  # 20% max correction

                base_speed = (
                    speed if abs(error) > self.config.POSITION_THRESHOLD_CM else 0
                )
                return (base_speed - correction, base_speed + correction)

            else:
                self.state = {
                    MovementState.MOVING_Y: MovementState.ALIGNING_X,
                    MovementState.MOVING_X: MovementState.FINAL_ROTATION,
                }[self.state]
                return 0, 0

        return 0, 0

    def execute_instruction(self, instructions: str):
        list_movements = parse_instructions(instructions)
        for movement in list_movements:
            type, length = movement
            if type == "r":
                self.move_distance(length)
            elif type == "a":
                self.turn_angle(length)

    def at_target(self) -> bool:
        """Check if robot has reached target position and orientation"""
        return self.state == MovementState.DONE
