from typing import Optional, Tuple
import time
import numpy as np
from dataclasses import dataclass
from models.position import Position
from config import ControlConfig


@dataclass
class PositionError:
    """Represents linear and angular errors for PID control"""

    linear: float  # Linear error in meters
    angular: float  # Angular error in radians


class PositionController:
    """
    High-level position controller using camera feedback
    Implements PID control for both linear and angular movements
    """

    def __init__(self, config: ControlConfig):
        self.config = config

        # Controller states
        self.current_pose: Optional[Position] = None
        self.target_pose = Position(
            x=0.0, y=0.0, theta=0.0, camera_angle=0.0, confidence=1.0
        )  # Default target at origin
        self.last_control_time = time.time()

        # PID control states
        self.linear_integral = 0.0
        self.angular_integral = 0.0
        self.last_error: Optional[PositionError] = None

        # Position tracking
        self.last_position_update = 0.0
        self.position_timeout = 0.5  # seconds

        # Control parameters
        self.max_linear_velocity = 0.5  # m/s
        self.max_angular_velocity = 1.0  # rad/s
        self.heading_threshold = 0.1  # radians
        self.position_threshold = 0.05  # meters
        self.min_confidence = 0.3  # Minimum confidence to consider position valid

    def update_position(self, new_pose: Position) -> None:
        """
        Update current position from vision system
        Args:
            new_pose: New position from vision system
        """
        # Only update if confidence is sufficient
        if new_pose.confidence >= self.min_confidence:
            self.current_pose = new_pose
            self.last_position_update = time.time()

    def get_current_pose(self) -> Optional[Position]:
        """
        Get current position if available and valid
        Returns:
            Current position or None if data is invalid/expired
        """
        if self.current_pose is None:
            return None

        # Check if position data is too old
        if time.time() - self.last_position_update > self.position_timeout:
            return None

        # Check confidence
        if self.current_pose.confidence < self.min_confidence:
            return None

        return self.current_pose

    def set_target_pose(self, target: Position) -> None:
        """
        Set new target position and reset PID states
        Args:
            target: Target position to reach
        """
        self.target_pose = target
        # Reset PID states for new target
        self.linear_integral = 0.0
        self.angular_integral = 0.0
        self.last_error = None

    def get_target_pose(self) -> Position:
        """Get current target position"""
        return self.target_pose

    def compute_pose_error(self, current: Position, target: Position) -> PositionError:
        """
        Compute linear and angular errors between current and target pose
        Args:
            current: Current position
            target: Target position
        Returns:
            PositionError containing linear and angular errors
        """
        # Compute position error
        dx = target.x - current.x
        dy = target.y - current.y
        linear_error = np.sqrt(dx * dx + dy * dy)

        # Compute desired heading to target
        desired_theta = np.arctan2(dy, dx)

        # Consider camera angle in error computation if available
        effective_theta = current.theta
        if current.camera_angle is not None:
            effective_theta += current.camera_angle

        # Compute angular error (normalized to [-pi, pi])
        angular_error = desired_theta - effective_theta
        angular_error = np.arctan2(np.sin(angular_error), np.cos(angular_error))

        return PositionError(linear_error, angular_error)

    def compute_control(self, current_pos: Position) -> Optional[Tuple[float, float]]:
        """
        Compute wheel velocities using PID control
        Args:
            current_pos: Current robot position
        Returns:
            Tuple of (left_velocity, right_velocity) or None if control update not needed
        """
        current_time = time.time()
        dt = current_time - self.last_control_time

        # Check if enough time has passed for control update
        if dt < self.config.POSITION_PERIOD:
            return None

        # Scale control based on confidence
        confidence_factor = current_pos.confidence

        # Compute current error
        error = self.compute_pose_error(current_pos, self.target_pose)

        if self.last_error is None:
            self.last_error = error

        # Update integrals with anti-windup
        self.linear_integral = np.clip(
            self.linear_integral + error.linear * dt,
            -self.config.POSITION_MAX_INTEGRAL,
            self.config.POSITION_MAX_INTEGRAL,
        )
        self.angular_integral = np.clip(
            self.angular_integral + error.angular * dt,
            -self.config.POSITION_MAX_INTEGRAL,
            self.config.POSITION_MAX_INTEGRAL,
        )

        # Compute derivatives
        linear_derivative = (error.linear - self.last_error.linear) / dt
        angular_derivative = (error.angular - self.last_error.angular) / dt

        # Compute PID outputs with confidence scaling
        linear_control = confidence_factor * (
            self.config.POSITION_Kp * error.linear
            + self.config.POSITION_Ki * self.linear_integral
            + self.config.POSITION_Kd * linear_derivative
        )

        angular_control = confidence_factor * (
            self.config.POSITION_Kp * error.angular
            + self.config.POSITION_Ki * self.angular_integral
            + self.config.POSITION_Kd * angular_derivative
        )

        # Update control states
        self.last_error = error
        self.last_control_time = current_time

        # Convert to wheel velocities
        return self.control_to_wheel_velocities(linear_control, angular_control)

    def control_to_wheel_velocities(
        self, linear_control: float, angular_control: float
    ) -> Tuple[float, float]:
        """
        Convert linear and angular control signals to wheel velocities
        Args:
            linear_control: Linear velocity control signal
            angular_control: Angular velocity control signal
        Returns:
            Tuple of left and right wheel velocities
        """
        # Apply velocity limits
        linear_control = np.clip(
            linear_control, -self.max_linear_velocity, self.max_linear_velocity
        )
        angular_control = np.clip(
            angular_control, -self.max_angular_velocity, self.max_angular_velocity
        )

        # Convert to differential drive commands
        left_velocity = linear_control - angular_control
        right_velocity = linear_control + angular_control

        # Normalize to maintain maximum velocities
        max_velocity = max(abs(left_velocity), abs(right_velocity))
        if max_velocity > 1.0:
            left_velocity /= max_velocity
            right_velocity /= max_velocity

        return left_velocity, right_velocity

    def at_target(self) -> bool:
        """
        Check if robot has reached target position
        Returns:
            True if robot is at target position and orientation
        """
        if self.current_pose is None:
            return False

        error = self.compute_pose_error(self.current_pose, self.target_pose)
        return (
            abs(error.linear) < self.position_threshold
            and abs(error.angular) < self.heading_threshold
        )
