from .pid_controller import PIDController
from library_motor.controller import Controller
import time
from typing import Tuple
from config import ControlConfig


class MotorsController:
    def __init__(self):
        self.controller = Controller()
        self.controller.set_motor_shutdown_timeout(5)

        # Constants for movement calculations
        self.TICKS_PER_CM = 174.7  # encoder ticks per centimeter
        self.TICKS_PER_RAD = 1543  # encoder ticks per radian
        self.MAX_SPEED = 100

    def standby(self):
        """Stop all motors."""
        self.controller.standby()

    def move_uncontrolled(self, direction: str, speed):
        """Basic movement without position control."""
        speed = -speed

        if direction == "forward":
            self.controller.set_motor_speed(speed, speed)
        elif direction == "backward":
            self.controller.set_motor_speed(-speed, -speed)
        elif direction == "right":
            self.controller.set_motor_speed(speed, -speed)
        elif direction == "left":
            self.controller.set_motor_speed(-speed, speed)
        elif direction == "stop":
            self.standby()

    def move_controlled(
        self,
        distance: float,
        direction: int = 1,
        speed: int = 50,
        delta_time: int = 20,
    ) -> Tuple[int, int]:
        """Controlled movement with position feedback."""
        if direction == 0:
            return (0, 0)

        if distance < 0.0:
            distance = -distance
            direction = -1

        # Calculate target distance in encoder ticks
        target_ticks = int(distance * self.TICKS_PER_CM)
        remaining_left = remaining_right = target_ticks

        # Setup PID controller
        dt = delta_time * 0.001  # convert to seconds
        # time_ratio = int(dt / 0.01)

        pid = PIDController(
            kp=1.0,
            ki=0.1,
            kd=0.05,
            setpoint=0,
            min_value=-100 + speed,  # -0.5 * speed * time_ratio,
            max_value=100 - speed,  # 0.5 * speed * time_ratio,
        )

        correction = 0
        speed_oriented = -direction * speed

        # Clear encoder counts
        self.controller.get_encoder_ticks()

        while True:
            overshoot_interval = 2 * (speed + abs(correction)) * int(dt / 0.01)

            if (
                remaining_left < overshoot_interval
                or remaining_right < overshoot_interval
            ):
                speed //= 3
                speed_oriented //= 3

            if speed <= 2:
                break

            self.controller.set_motor_speed(
                speed_oriented + correction, speed_oriented - correction
            )

            time.sleep(dt)

            ticks = self.controller.get_encoder_ticks()

            remaining_left -= -direction * ticks[0]

            remaining_right -= -direction * ticks[1]

            error = (remaining_left - remaining_right) * 0.01 / dt

            correction = pid.compute(error, dt)

        self.standby()

        time.sleep(0.5)

        return (remaining_left, remaining_right)

    def turn_controlled(
        self, angle: float, side: int = 1, speed: int = 20, delta_time: int = 20
    ) -> Tuple[int, int]:
        """Controlled rotation with position feedback."""
        if side == 0:
            return (0, 0)

        if angle < 0:
            side = -1
            angle = -angle

        # Calculate target angle in encoder ticks
        target_ticks = int(angle * self.TICKS_PER_RAD)
        remaining_left = remaining_right = target_ticks

        # Setup PID controller
        dt = delta_time * 0.001

        pid = PIDController(
            kp=1.0,
            ki=0.1,
            kd=0.05,
            setpoint=0,
            min_value=-100,
            max_value=100,
        )

        correction = 0
        speed_oriented = side * speed

        # Clear encoder counts
        self.controller.get_encoder_ticks()

        while True:
            overshoot_interval = 2 * (speed + abs(correction)) * int(dt / 0.01)

            if (
                remaining_left < overshoot_interval
                or remaining_right < overshoot_interval
            ):
                speed //= 3
                speed_oriented //= 3

            if speed <= 2:
                break

            self.controller.set_motor_speed(
                -speed_oriented + correction, speed_oriented + correction
            )

            time.sleep(dt)

            ticks = self.controller.get_encoder_ticks()

            remaining_left -= -side * ticks[0]

            remaining_right -= side * ticks[1]

            error = (remaining_left - remaining_right) * 0.01 / dt

            correction = side * pid.compute(error, dt)

        self.standby()
        time.sleep(0.5)
        return (remaining_left, remaining_right)

    def get_speed(self) -> Tuple[int, int]:
        """Get current motor speeds."""
        return self.controller.get_motor_speed()
