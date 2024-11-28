import time
import numpy as np


class VelocityPIDController:
    """PID controller for motor velocity control"""

    def __init__(self, kp: float, ki: float, kd: float, max_integral: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_integral = max_integral

        self.integral = 0.0
        self.last_error = None
        self.last_time = time.time()

    def compute(self, target: float, current: float) -> float:
        """Compute PID control value"""
        current_time = time.time()
        dt = current_time - self.last_time

        error = target - current

        # Proportional term
        p_term = self.kp * error

        # Integral term with anti-windup
        self.integral = np.clip(
            self.integral + error * dt, -self.max_integral, self.max_integral
        )
        i_term = self.ki * self.integral

        # Derivative term
        if self.last_error is not None:
            d_term = self.kd * (error - self.last_error) / dt
        else:
            d_term = 0

        self.last_error = error
        self.last_time = current_time

        return p_term + i_term + d_term

    def reset(self):
        """Reset controller state"""
        self.integral = 0.0
        self.last_error = None
