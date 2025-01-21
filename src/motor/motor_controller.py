from src.utils.pid_controller import PIDController
from src.utils.thread_safe_dequeue import ThreadSafeDeque
from src.config import PIDConfig, MotorConfig, RobotDimensions, RateConfig
from .odometry_controller import OdometryController
import time as t
import threading, queue, argparse

from numpy import pi

# import web_pdb

# debug
# web_pdb.set_trace(host="0.0.0.0", port=8080)


def init_pid(
    target: int = 0, min=-MotorConfig.MAX_SPEED.value, max=MotorConfig.MAX_SPEED.value
):
    k_p = PIDConfig.K_P.value
    k_i = PIDConfig.K_I.value
    k_d = PIDConfig.K_D.value
    return PIDController(target, min, max, k_p, k_i, k_d)


class MotorController:
    def __init__(
        self, odometry_controller: OdometryController, test_mode: bool = False
    ):
        self.test_mode = test_mode
        self.odometry = odometry_controller
        self.odometry_ticks = (0, 0)
        self.raw_speed: int = 50
        self.speed: int = 0
        self.pid = None
        self.error = (0, 0)
        self.current_mode = "manual"
        self.update_frequency = RateConfig.MOTOR_FREQUENCY.value
        self.command_queue = queue.Queue()
        self.worker_thread = threading.Thread(
            target=self._command_processor, daemon=True
        )
        self.worker_thread.start()
        self.stop_event = threading.Event()
        self.terminate_event = threading.Event()

        if not test_mode:
            from .libMotors import controller as c

            self.controller = c.Controller()
            self.controller.set_motor_shutdown_timeout(1)
            self.controller.get_encoder_ticks()  # to init the ticks counter

    def _command_processor(self):
        while True:
            item = self.command_queue.get()
            if item != ():
                # log
                print("poped out command")
                command, args = item
                command(*args)
                try:
                    self.command_queue.task_done()
                except ValueError:
                    # log
                    print("Got value error")
            if self.terminate_event.is_set():
                print("Terminate event is set, finishing...")
                self.clear_command_queue()
                break

    def update_raw_speed(self, speed):
        try:
            if (
                type(speed) != int
                or speed < MotorConfig.MIN_RAW_SPEED.value
                or speed > MotorConfig.MAX_RAW_SPEED.value
            ):
                raise ValueError(
                    f"Raw speed should be a positive int between {MotorConfig.MIN_RAW_SPEED.value} and {MotorConfig.MAX_RAW_SPEED.value}, -{speed}- was provided"
                )
            else:
                self.raw_speed = speed
        except ValueError as e:
            print(f"ERROR - {e}")
        return

    def update_speed(self, speed):
        try:
            if (
                type(speed) != int
                or speed < MotorConfig.MIN_SPEED.value
                or speed > MotorConfig.MAX_SPEED.value
            ):
                raise ValueError(
                    f"Speed should be a positive int between {MotorConfig.MIN_SPEED.value} and {MotorConfig.MAX_SPEED.value}, -{speed}- was provided"
                )
            else:
                self.speed = speed
        except ValueError as e:
            print(f"ERROR - {e}")
        return

    def update_odometry(self):
        self.odometry.update_position_from_ticks(*self.odometry_ticks)
        self.odometry_ticks = (0, 0)

    def move_uncontrolled(self, direction: str, speed=None):
        """Basic movement without control."""

        if speed is not None:
            self.update_raw_speed(speed)
        else:
            self.update_raw_speed(MotorConfig.DEFAULT_RAW_SPEED.value)
        speed = -self.raw_speed

        if self.test_mode:
            print(
                f"TEST - Robot should move uncontrolled towards direction {direction} at speed {self.raw_speed}"
            )
            return

        if direction == "forward":
            self.controller.set_raw_motor_speed(speed, speed)
        elif direction == "backward":
            self.controller.set_raw_motor_speed(-speed, -speed)
        elif direction == "right":
            self.controller.set_raw_motor_speed(-speed, speed)
        elif direction == "left":
            self.controller.set_raw_motor_speed(speed, -speed)
        elif direction == "stop":
            self.controller.set_raw_motor_speed(0, 0)
        else:
            print(f"ERROR - Provided direction ({direction}) is not supported.")

    def move_controlled(self, distance: float, **kwargs):
        """Controlled movement with position feedback."""
        speed = kwargs.get("speed", None)
        no_wait = kwargs.get("no_wait", False)
        event = kwargs.get("event", None)

        def command(distance, speed, event):
            if event:
                event.clear()
            if distance == 0:
                return (0, 0)
            elif distance < 0.0:
                distance = -distance
                direction = -1
            else:
                direction = 1
            if speed is not None:
                self.update_speed(speed)
            else:
                self.update_speed(MotorConfig.DEFAULT_MOVING_SPEED.value)

            if self.test_mode:
                if event:
                    event.set()
                print(
                    f"TEST - Robot should move controlled at speed {self.speed} for {direction * distance}m."
                )
                return

            target_ticks = int(distance * RobotDimensions.TICKS_PER_METER.value)
            remaining_left = remaining_right = target_ticks

            motor_rate = 1 / RateConfig.MOTOR_FREQUENCY.value
            odometry_rate = 1 / RateConfig.ODOMETRY_FREQUENCY.value

            pid = init_pid(
                0,
                -MotorConfig.MAX_SPEED.value - self.speed,
                MotorConfig.MAX_SPEED.value - self.speed,
            )

            correction = 0
            speed_oriented = -direction * self.speed

            # Clear encoder counts
            self.controller.get_encoder_ticks()

            next_update_odometry = t.time() + odometry_rate
            next_update_motor = t.time() + motor_rate

            while not self.stop_event.is_set() and not self.terminate_event.is_set():
                overshoot_interval = (
                    2 * 100 * motor_rate * (self.speed + abs(correction))
                )

                # # log
                # print(
                #     f"update_control at {t.time()} : {overshoot_interval}, {remaining_left}, {remaining_right}, {correction}"
                # )

                if (
                    remaining_left < overshoot_interval
                    or remaining_right < overshoot_interval
                ):
                    self.speed //= 3
                    speed_oriented = -direction * self.speed

                if self.speed <= 2:
                    break

                self.controller.set_motor_speed(
                    speed_oriented + correction, speed_oriented - correction
                )

                t.sleep(next_update_motor - t.time())
                next_update_motor += motor_rate

                ticks = self.controller.get_encoder_ticks()

                self.odometry_ticks = (
                    self.odometry_ticks[0] + ticks[0],
                    self.odometry_ticks[1] + ticks[1],
                )
                if t.time() > next_update_odometry:
                    self.update_odometry()
                    next_update_odometry += odometry_rate

                remaining_left -= -direction * ticks[0]

                remaining_right -= -direction * ticks[1]

                error = (remaining_left - remaining_right) * 0.01 / motor_rate

                correction = direction * pid.compute(error, motor_rate)

            self.controller.standby()
            self.update_odometry()

            t.sleep(0.3)

            if event:
                event.set()
            return (remaining_left, remaining_right)

        item = (command, (distance, speed, event))
        if no_wait:
            print("no wait is not implemented yet...")
        else:
            self.command_queue.put(item)

    def turn_controlled(self, angle: float, **kwargs):
        """Controlled rotation with position feedback."""
        speed = kwargs.get("speed", None)
        no_wait = kwargs.get("no_wait", False)
        event = kwargs.get("event", None)

        def command(angle, speed, event):
            if event:
                event.clear()
            if angle > pi:
                angle = -(2 * pi - angle)
            if angle == 0.0:
                return (0, 0)
            elif angle < 0.0:
                angle = -angle
                side = -1
            else:
                side = 1

            if speed is not None:
                self.update_speed(speed)
            else:
                self.update_speed(MotorConfig.DEFAULT_ROTATING_SPEED.value)

            if self.test_mode:
                if event:
                    event.set()
                print(
                    f"TEST - Robot should turn controlled {side * angle}rad at speed {self.speed}."
                )
                return
            target_ticks = int(angle * RobotDimensions.TICKS_PER_RAD.value)
            remaining_left = remaining_right = target_ticks

            motor_rate = 1 / RateConfig.MOTOR_FREQUENCY.value
            odometry_rate = 1 / RateConfig.ODOMETRY_FREQUENCY.value

            pid = init_pid(
                0,
                -MotorConfig.MAX_SPEED.value - self.speed,
                MotorConfig.MAX_SPEED.value - self.speed,
            )

            correction = 0
            speed_oriented = side * self.speed

            # Clear encoder counts
            self.controller.get_encoder_ticks()

            next_update_odometry = t.time() + odometry_rate
            next_update_motor = t.time() + motor_rate
            correction = 0
            speed_oriented = side * self.speed

            while not self.stop_event.is_set() and not self.terminate_event.is_set():
                overshoot_interval = (
                    2 * 100 * motor_rate * (self.speed + abs(correction))
                )

                if (
                    remaining_left < overshoot_interval
                    or remaining_right < overshoot_interval
                ):
                    self.speed //= 3
                    speed_oriented = side * self.speed

                if self.speed <= 2:
                    break

                self.controller.set_motor_speed(
                    -speed_oriented + correction, speed_oriented + correction
                )

                t.sleep(next_update_motor - t.time())
                next_update_motor += motor_rate

                ticks = self.controller.get_encoder_ticks()

                self.odometry_ticks = (
                    self.odometry_ticks[0] + ticks[0],
                    self.odometry_ticks[1] + ticks[1],
                )
                if t.time() > next_update_odometry:
                    self.update_odometry()
                    next_update_odometry += odometry_rate

                remaining_left -= -side * ticks[0]

                remaining_right -= side * ticks[1]

                error = (remaining_left - remaining_right) * 0.01 / motor_rate

                correction = side * pid.compute(error, motor_rate)

            self.controller.standby()
            self.update_odometry()

            t.sleep(0.3)

            if event:
                event.set()
            return (remaining_left, remaining_right)

        item = (command, (angle, speed, event))
        if no_wait:
            print("no_wait not implemented yet...")
        else:
            self.command_queue.put(item)

    def get_speed(self):
        """Get current motor speeds."""
        return self.controller.get_motor_speed()

    def delay_controlled(self, delay, **kwargs):
        """Add a delay between movements."""
        no_wait = kwargs.get("no_wait", False)
        event = kwargs.get("event", None)

        def command(delay, event):
            start_time = t.time()
            if event:
                event.clear()
            while True:
                remaining = delay - (t.time() - start_time)
                if remaining <= 0:
                    break
                if self.stop_event.wait(timeout=remaining) or self.terminate_event.wait(
                    timeout=remaining
                ):
                    break
            event.set()

        item = (command, (delay, event))
        if no_wait:
            print("not yet implemented")
            # self.command_queue.appendleft(item)
        else:
            self.command_queue.put(item)

    def turn_controlled_deg(self, angle, **kwargs):
        speed = kwargs.get("speed", None)
        no_wait = kwargs.get("no_wait", False)
        event = kwargs.get("event", None)
        angle *= pi / 180
        self.turn_controlled(angle, speed=speed, no_wait=no_wait, event=event)

    def face_controlled(self, orientation, **kwargs):
        speed = kwargs.get("speed", None)
        no_wait = kwargs.get("no_wait", False)
        event = kwargs.get("event", None)
        orientation %= 2 * pi
        angle = orientation - self.odometry.get_position()[2]
        self.turn_controlled(angle, speed=speed, no_wait=no_wait, event=event)

    def face_controlled_deg(self, orientation, **kwargs):
        speed = kwargs.get("speed", None)
        no_wait = kwargs.get("no_wait", False)
        event = kwargs.get("event", None)
        self.face_controlled(
            orientation * pi / 180, speed=speed, no_wait=no_wait, event=event
        )

    def move_controlled_centimeters(self, distance, **kwargs):
        speed = kwargs.get("speed", None)
        no_wait = kwargs.get("no_wait", False)
        event = kwargs.get("event", None)
        distance /= 100
        self.move_controlled(distance, speed=speed, no_wait=no_wait, event=event)

    def terminate_controlled(self):
        print("Interrupting all the ongoing controlled movements...")
        self.terminate_event.set()
        self.stop_event.set()

    def clear_command_queue(self, immediate=False):
        if immediate:
            self.stop_event.set()
        self.command_queue.shutdown(immediate=True)
        self.command_queue = queue.Queue()
        self.stop_event.clear()

    def _target_started(self, event):
        def command(event):
            event.set()

        self.command_queue.put((command, (event,)))

    def _target_achivied(self, event):

        def command(event):
            event.clear()

        self.command_queue.put((command, (event,)))

    def execute_instructions(self, instructions):
        """Translate a list of instructions into movement executions."""
        instruction_list = instructions.split(",")
        count = 0
        print(f"Instructions List : {instruction_list}")
        for instruction in instruction_list:
            instruction = instruction.strip()
            if instruction.startswith("a"):
                try:
                    angle = float(instruction[1:])
                    self.turn_controlled_deg(angle)
                except ValueError:
                    print(f"Invalid angle value in instruction: {instruction}")
            elif instruction.startswith("f"):
                try:
                    orientation = float(instruction[1:])
                    self.face_controlled_deg(orientation)
                except ValueError:
                    print(f"Invalid orientation value in instruction: {instruction}")
            elif instruction.startswith("r"):
                try:
                    distance = float(instruction[1:])
                    self.move_controlled_centimeters(distance)
                except ValueError:
                    print(f"Invalid distance value in instruction: {instruction}")
                    print("Queued moving instruction...")
            elif instruction.startswith("d"):
                try:
                    delay = float(instruction[1:])
                    self.delay_controlled(delay)
                    print("Queued delay instruction...")
                except ValueError:
                    print(f"Invalid delay value in instruction: {instruction}")
            else:
                print(f"Unknown instruction: {instruction}")

    def shutdown(self):
        """Gracefully stop the worker thread and wait for it to finish."""
        print("Shutting down worker thread...")
        self.command_queue.join()
        print(self.command_queue)
        self.terminate_event.set()
        self.command_queue.put(())  # Ensure the queue isn't blocking
        self.worker_thread.join()  # Wait for the thread to finish
        print("Worker thread shut down successfully.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        prog="MotorController",
        description="Motor Controller CLI",
    )
    parser.add_argument(
        "-i",
        "--instructions",
        type=str,
        default=MotorConfig.DEFAULT_INSTR,
        help="Instruction string to execute",
    )
    args = parser.parse_args()

    odo = OdometryController(0, 0, 0)
    motor_controller = MotorController(odo)
    print("Starting executing instructions...")
    instr = args.instructions
    # # log
    # print("Executing the following instructions: ", instr)
    motor_controller.execute_instructions(instr)
    # motor_controller.shutdown()
    print(odo.x * 100, odo.y * 100, odo.orientation * 180 / pi)
