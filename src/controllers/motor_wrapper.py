from typing import Tuple
import time
from dataclasses import dataclass
from config import ControlConfig


@dataclass
class SimulatedMotorState:
    """State for simulated motor"""

    pwm: float = 0.0  # Raw PWM value (-1.0 to 1.0)
    ticks_per_sec: int = 0  # Current ticks per second
    total_ticks: int = 0  # Accumulated ticks
    last_update: float = 0.0


class MotorWrapper:
    """Wrapper for motor controller with tick-based interface"""

    def __init__(self, config: ControlConfig, test_mode: bool = False):
        self.config = config
        self.test_mode = test_mode
        self.ticks_per_rotation = config.TICKS_PER_ROTATION
        self.max_ticks_per_sec = config.MAX_TICKS_PER_SEC

        if test_mode:
            self.left_motor = SimulatedMotorState()
            self.right_motor = SimulatedMotorState()
        else:
            try:
                from ..library_motor.controller import Controller

                self.controller = Controller(config.I2C_BUS)
                self.relative = self.controller.new_relative()
                self.setup_controller()
            except ImportError as e:
                print(f"Error: motor_controller module not found: {e}")
                self.test_mode = True
                self.left_motor = SimulatedMotorState()
                self.right_motor = SimulatedMotorState()

    def setup_controller(self) -> None:
        """Initial setup of the controller board"""
        if not self.test_mode:
            # Convert PID coefficients to controller's scale
            self.controller.set_pid_coefficients(
                self.config.MOTOR_Kp / 256,
                self.config.MOTOR_Ki / 256,
                self.config.MOTOR_Kd / 256,
            )
            self.controller.set_pwm_frequency(20000)  # 20kHz PWM
            self.controller.set_motor_shutdown_timeout(0.5)
            self.controller.set_encoder_mode(self.config.ENCODER_MODE)

    def simulate_motor_physics(self, motor: SimulatedMotorState, dt: float) -> int:
        """Improved physics simulation for test mode"""
        # Model motor response with acceleration limits
        max_accel = self.max_ticks_per_sec / 0.5  # Reach max speed in 0.5s
        target_ticks_per_sec = int(motor.pwm * self.max_ticks_per_sec)

        # Calculate acceleration
        current_speed = motor.ticks_per_sec
        speed_diff = target_ticks_per_sec - current_speed
        accel = max(min(speed_diff / dt, max_accel), -max_accel)

        # Update speed and position
        motor.ticks_per_sec += int(accel * dt)
        motor.total_ticks += int(motor.ticks_per_sec * dt)
        motor.last_update = time.time()

        return motor.ticks_per_sec

    def get_motor_ticks(self) -> Tuple[int, int]:
        """Get current ticks per second for each motor"""
        if self.test_mode:
            current_time = time.time()
            dt = current_time - self.left_motor.last_update

            left_ticks = self.simulate_motor_physics(self.left_motor, dt)
            right_ticks = self.simulate_motor_physics(self.right_motor, dt)

            return left_ticks, right_ticks
        else:
            speeds = self.controller.get_motor_speed()
            return speeds if speeds is not None else (0, 0)

    def set_motor_pwm(self, left_pwm: float, right_pwm: float) -> None:
        """Set raw PWM values (-1.0 to 1.0)"""
        # Ensure PWM values are within bounds
        left_pwm = max(min(left_pwm, 1.0), -1.0)
        right_pwm = max(min(right_pwm, 1.0), -1.0)

        if self.test_mode:
            self.left_motor.pwm = left_pwm
            self.right_motor.pwm = right_pwm
        else:
            # Convert to controller's PWM range (-127 to 127)
            left = int(left_pwm * 127)
            right = int(right_pwm * 127)
            self.controller.set_raw_motor_speed(left, right)

    def set_motor_ticks(self, left_ticks: int, right_ticks: int) -> None:
        """Set target ticks per second for each motor"""
        # Apply speed limits
        left_ticks = max(
            min(left_ticks, self.max_ticks_per_sec), -self.max_ticks_per_sec
        )
        right_ticks = max(
            min(right_ticks, self.max_ticks_per_sec), -self.max_ticks_per_sec
        )

        if self.test_mode:
            # Convert target ticks to normalized PWM for simulation
            self.left_motor.pwm = left_ticks / self.max_ticks_per_sec
            self.right_motor.pwm = right_ticks / self.max_ticks_per_sec
        else:
            self.controller.set_motor_speed(left_ticks, right_ticks)

    def stop(self) -> None:
        """Stop all motors"""
        if self.test_mode:
            self.left_motor.pwm = 0.0
            self.right_motor.pwm = 0.0
            self.left_motor.ticks_per_sec = 0
            self.right_motor.ticks_per_sec = 0
        else:
            self.controller.standby()
