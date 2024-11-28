from typing import Tuple
from config import ControlConfig
from .motor_wrapper import MotorWrapper
from .pid_controller import VelocityPIDController


class MotorController:
    """Motor controller implementing cascaded control"""

    def __init__(self, config: ControlConfig, test_mode: bool = False):
        self.config = config
        self.motor_wrapper = MotorWrapper(config, test_mode)
        self.target_velocities = (0, 0)

        # Initialize velocity PID controllers
        self.left_pid = VelocityPIDController(
            kp=config.MOTOR_Kp,
            ki=config.MOTOR_Ki,
            kd=config.MOTOR_Kd,
            max_integral=config.MOTOR_MAX_INTEGRAL,
        )
        self.right_pid = VelocityPIDController(
            kp=config.MOTOR_Kp,
            ki=config.MOTOR_Ki,
            kd=config.MOTOR_Kd,
            max_integral=config.MOTOR_MAX_INTEGRAL,
        )

        # Control mode flag
        self.use_internal_pid = True  # Set to False to use external PID

    def update_velocity_control(self):
        """Update motor control"""
        if self.use_internal_pid:
            # Use controller board's built-in PID
            self.motor_wrapper.set_motor_velocities(*self.target_velocities)
        else:
            # Use our own PID implementation
            current_velocities = self.motor_wrapper.get_motor_velocities()

            # Compute PID output for each motor
            left_pwm = self.left_pid.compute(
                self.target_velocities[0], current_velocities[0]
            )
            right_pwm = self.right_pid.compute(
                self.target_velocities[1], current_velocities[1]
            )

            # Apply PWM to motors
            self.motor_wrapper.set_motor_pwm(left_pwm, right_pwm)

    def set_target_velocities(self, left_velocity: float, right_velocity: float):
        """Set target velocities"""
        self.target_velocities = (left_velocity, right_velocity)

    def stop(self):
        """Stop motors and reset controllers"""
        self.motor_wrapper.stop()
        self.left_pid.reset()
        self.right_pid.reset()
