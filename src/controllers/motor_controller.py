from config import ControlConfig
from .motor_wrapper import MotorWrapper
from .pid_controller import VelocityPIDController
import time


class MotorController:
    """Motor controller implementing cascaded control with tick-based interface"""

    def __init__(self, config: ControlConfig, test_mode: bool = False):
        self.config = config
        self.motor_wrapper = MotorWrapper(config, test_mode)
        self.target_ticks = (0, 0)  # Target ticks per second for each motor

        # Initialize velocity PID controllers with proper scaling
        self.left_pid = VelocityPIDController(
            kp=config.MOTOR_Kp / config.TICKS_PER_CM,  # Scale for ticks
            ki=config.MOTOR_Ki / config.TICKS_PER_CM,
            kd=config.MOTOR_Kd / config.TICKS_PER_CM,
            max_integral=config.MOTOR_MAX_INTEGRAL * config.TICKS_PER_CM,
        )
        self.right_pid = VelocityPIDController(
            kp=config.MOTOR_Kp / config.TICKS_PER_CM,
            ki=config.MOTOR_Ki / config.TICKS_PER_CM,
            kd=config.MOTOR_Kd / config.TICKS_PER_CM,
            max_integral=config.MOTOR_MAX_INTEGRAL * config.TICKS_PER_CM,
        )

        self.use_internal_pid = True
        self.last_update_time = time.time()

    def update_velocity_control(self) -> None:
        """Update motor control using ticks per second"""
        current_time = time.time()
        dt = current_time - self.last_update_time

        if dt < self.config.MOTOR_PERIOD:
            return

        if self.use_internal_pid:
            self.motor_wrapper.set_motor_ticks(*self.target_ticks)
        else:
            current_ticks = self.motor_wrapper.get_motor_ticks()

            # Compute PID output and convert to PWM
            left_pwm = self.left_pid.compute(self.target_ticks[0], current_ticks[0], dt)
            right_pwm = self.right_pid.compute(self.target_ticks[1], current_ticks[1])

            # Apply PWM limits
            left_pwm = max(min(left_pwm, 1.0), -1.0)
            right_pwm = max(min(right_pwm, 1.0), -1.0)

            self.motor_wrapper.set_motor_pwm(left_pwm, right_pwm)

        self.last_update_time = current_time

    def set_target_ticks(self, left_ticks: int, right_ticks: int) -> None:
        """Set target velocities in ticks per second"""
        # Apply speed limits
        self.target_ticks = (
            max(
                min(left_ticks, self.config.MAX_TICKS_PER_SEC),
                -self.config.MAX_TICKS_PER_SEC,
            ),
            max(
                min(right_ticks, self.config.MAX_TICKS_PER_SEC),
                -self.config.MAX_TICKS_PER_SEC,
            ),
        )

    def stop(self) -> None:
        """Stop motors and reset controllers"""
        self.target_ticks = (0, 0)
        self.motor_wrapper.stop()
        self.left_pid.reset()
        self.right_pid.reset()
