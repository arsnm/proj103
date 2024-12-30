import time
import numpy as np


class PIDController:
    def __init__(self, kp: float, ki: float, kd: float, max_integral: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_integral = max_integral
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.last_time = time.time()

    def compute(self, target: float, current: float) -> float:
        now = time.time()
        dt = now - self.last_time
        if dt <= 0:
            return 0.0

        error = target - current

        # Update integral with anti-windup
        self.integral = np.clip(
            self.integral + error * dt, -self.max_integral, self.max_integral
        )

        # Calculate derivative
        derivative = (error - self.prev_error) / dt if dt > 0 else 0

        # Compute control output
        output = self.kp * error + self.ki * self.integral + self.kd * derivative

        # Update state
        self.prev_error = error
        self.last_time = now

        return output
