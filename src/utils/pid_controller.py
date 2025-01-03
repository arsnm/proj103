from dataclasses import dataclass


@dataclass
class PIDController:
    setpoint: float
    min_value: float
    max_value: float
    kp: float
    ki: float
    kd: float

    def __post_init__(self):
        self.previous_error = 0.0
        self.integral = 0.0

    def compute(self, current_value: float, delta_time: float) -> float:
        error = self.setpoint - current_value

        # Proportional term
        p_term = self.kp * error

        # Integral term
        self.integral += error * delta_time
        i_term = self.ki * self.integral

        # Derivative term (if delta_time is 0, skip to avoid division by zero)
        if delta_time > 0:
            d_term = self.kd * (error - self.previous_error) / delta_time
        else:
            d_term = 0

        self.previous_error = error

        print(f"error: {error}, {p_term}, {i_term}, {d_term}")
        # Calculate output and apply limits
        output = int(p_term + i_term + d_term)
        return max(int(self.min_value), min(int(self.max_value), output))
