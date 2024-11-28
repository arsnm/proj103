from typing import Tuple
import time
from dataclasses import dataclass
from config import ControlConfig


@dataclass
class SimulatedMotorState:
    """State for simulated motor"""

    pwm: float = 0.0
    velocity: float = 0.0
    position: float = 0.0
    last_update: float = 0.0


class MotorWrapper:
    """Wrapper for motor controller with test mode support"""

    def __init__(self, config: ControlConfig, test_mode: bool = False):
        self.config = config
        self.test_mode = test_mode

        if test_mode:
            print("[TEST MODE] Motor wrapper initialized")
            self.left_motor = SimulatedMotorState()
            self.right_motor = SimulatedMotorState()
        else:
            try:
                from ..library_motor.controller import Controller

                self.controller = Controller(8)  # i2c_bus = 8
                self.relative = self.controller.new_relative()
                self.setup_controller()
            except ImportError:
                print(
                    "Error: motor_controller module not found. Falling back to test mode."
                )
                self.test_mode = True
                self.left_motor = SimulatedMotorState()
                self.right_motor = SimulatedMotorState()

    def setup_controller(self):
        """Initial setup of the controller board"""
        if not self.test_mode:
            self.controller.set_pid_coefficients(
                self.config.MOTOR_Kp / 256,
                self.config.MOTOR_Ki / 256,
                self.config.MOTOR_Kd / 256,
            )
            self.controller.set_pwm_frequency(20000)
            self.controller.set_motor_shutdown_timeout(0.5)

    def simulate_motor_physics(self, motor: SimulatedMotorState):
        """Simple physics simulation for test mode"""
        current_time = time.time()
        dt = current_time - motor.last_update

        # Simple motor model: velocity follows PWM with some lag
        target_velocity = motor.pwm  # Assuming PWM -1 to 1 maps to velocity -1 to 1
        motor.velocity += (
            (target_velocity - motor.velocity) * dt * 2
        )  # Arbitrary response rate

        # Update position
        motor.position += motor.velocity * dt
        motor.last_update = current_time

        return motor.velocity

    def read_encoder_ticks(self) -> Tuple[int, int]:
        """Get encoder ticks since last read"""
        if self.test_mode:
            # Simulate encoder readings based on simulated velocity
            left_velocity = self.simulate_motor_physics(self.left_motor)
            right_velocity = self.simulate_motor_physics(self.right_motor)
            # Convert velocity to ticks (arbitrary scale for testing)
            ticks_scale = 100
            return (int(left_velocity * ticks_scale), int(right_velocity * ticks_scale))
        else:
            return self.controller.get_relative_encoder_ticks(self.relative)

    def set_motor_pwm(self, left_pwm: float, right_pwm: float):
        """Set raw PWM values (-1.0 to 1.0)"""
        if self.test_mode:
            self.left_motor.pwm = max(min(left_pwm, 1.0), -1.0)
            self.right_motor.pwm = max(min(right_pwm, 1.0), -1.0)
            print(
                f"[TEST MODE] Setting PWM - Left: {left_pwm:.2f}, Right: {right_pwm:.2f}"
            )
        else:
            left = int(max(min(left_pwm * 127, 127), -127))
            right = int(max(min(right_pwm * 127, 127), -127))
            self.controller.set_raw_motor_speed(left, right)

    def set_motor_velocities(self, left_velocity: float, right_velocity: float):
        """Set motor velocities"""
        if self.test_mode:
            # In test mode, directly set the target velocities
            self.left_motor.pwm = max(min(left_velocity, 1.0), -1.0)
            self.right_motor.pwm = max(min(right_velocity, 1.0), -1.0)
            print(
                f"[TEST MODE] Setting velocities - Left: {left_velocity:.2f}, Right: {right_velocity:.2f}"
            )
        else:
            left = int(left_velocity * 100)
            right = int(right_velocity * 100)
            self.controller.set_motor_speed(left, right)

    def get_motor_velocities(self) -> Tuple[float, float]:
        """Get current motor velocities"""
        if self.test_mode:
            left_velocity = self.simulate_motor_physics(self.left_motor)
            right_velocity = self.simulate_motor_physics(self.right_motor)
            return left_velocity, right_velocity
        else:
            speeds = self.controller.get_motor_speed()
            if speeds is None:
                return (0.0, 0.0)
            return (speeds[0] / 100, speeds[1] / 100)

    def stop(self):
        """Stop all motors"""
        if self.test_mode:
            self.left_motor.pwm = 0.0
            self.right_motor.pwm = 0.0
            print("[TEST MODE] Motors stopped")
        else:
            self.controller.standby()
