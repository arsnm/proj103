from src.utils.pid_controller import PIDController
from src.config import PIDConfig, MotorConfig, RobotDimensions, RateConfig
from .odometry_controller import OdometryController
import time as t
import threading, queue, argparse

from numpy import pi

# import web_pdb

# debug
# web_pdb.set_trace(host="0.0.0.0", port=8080)


class Command:
    def __init__(self, fun, desc="no desc", *args):
        self.fun = fun
        self.args = args
        self.desc = desc

    def __repr__(self):
        return self.desc


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
        self._queue_lock = threading.Lock()
        self.stop_command_event = threading.Event()
        self.terminate_event = threading.Event()

        # try:
        #     from .libMotors import controller as c
        #
        #     self.controller = c.Controller()
        #     self.controller.set_motor_shutdown_timeout(1)
        #     self.controller.get_encoder_ticks()  # to init the ticks counter
        # except ImportError:
        #     print("ERROR - smbus library not available, switching to test mode.")
        #     self.test_mode = True

        self.test_mode = test_mode

    def _command_processor(self):
        while True:
            try:
                item = self.command_queue.get()
                if type(item) == Command:

                    self.stop_command_event.clear()
                    # log
                    print("Poped out a command")
                    print(f"Description of the command {item.desc}")

                    ret = item.fun(*item.args)

                    if self.stop_command_event.is_set():
                        # log
                        print("Last command has been interrupted.")
                        self.stop_command_event.clear()

                    self.command_queue.task_done()
            except ValueError:
                pass

            if self.terminate_event.is_set():
                print("Terminate event is set, finishing...")
                self.terminate_event.clear()
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

    def update_timeout(self, timeout):
        self.controller.set_motor_shutdown_timeout(timeout)

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

    def _move_controlled(self, distance: float, speed, finish_event):
        """Controlled movement with position feedback."""

        if finish_event is not None:
            finish_event.clear()
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

        target_ticks = int(distance * RobotDimensions.TICKS_PER_METER.value)
        remaining_left = remaining_right = target_ticks

        if self.test_mode:
            print(
                f"TEST - Simulating following movement with a sleeping time proportional to distance"
            )
            sleep_time = int(distance / 0.11)
            print(f"TEST - Sleeping (**moving**) for {sleep_time}s")
            while True:
                if self.stop_command_event.wait(
                    timeout=sleep_time
                ) or self.terminate_event.wait(timeout=sleep_time):
                    break
                break
            self.odometry_ticks = (-direction * target_ticks, -direction * target_ticks)
            print(
                f"TEST - Robot should move controlled at speed {self.speed} for {direction * distance}m."
            )
            self.update_odometry()
            if finish_event is not None:
                finish_event.set()
            return (0, 0)

        else:
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

            while (
                not self.stop_command_event.is_set()
                and not self.terminate_event.is_set()
            ):
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

            if finish_event is not None:
                finish_event.set()
            return (remaining_left, remaining_right)

    def _turn_controlled(self, angle: float, speed, finish_event):
        """Controlled rotation with position feedback."""

        if finish_event is not None:
            finish_event.clear()

        # if angle > pi:
        #     angle = -(2 * pi - angle)

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

        target_ticks = int(angle * RobotDimensions.TICKS_PER_RAD.value)
        remaining_left = remaining_right = target_ticks

        if self.test_mode:
            print(
                f"TEST - Simulating following turn with a sleeping time proportional to angle"
            )
            sleep_time = int(angle / 0.63)
            print(f"TEST - Sleeping (**turning**) for {sleep_time}s")
            while True:
                if self.stop_command_event.wait(
                    timeout=sleep_time
                ) or self.terminate_event.wait(timeout=sleep_time):
                    break
                break
            self.odometry_ticks = (-side * target_ticks, side * target_ticks)
            print(
                f"TEST - Robot should turn controlled at speed {self.speed} for {side * angle}rad."
            )
            self.update_odometry()
            if finish_event is not None:
                finish_event.set()
            return (0, 0)

        else:

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

            while (
                not self.stop_command_event.is_set()
                and not self.terminate_event.is_set()
            ):
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

            if finish_event is not None:
                finish_event.set()
            return (remaining_left, remaining_right)

    def _delay_controlled(self, delay, finish_event):
        """Add a delay between movements."""

        if finish_event is not None:
            finish_event.clear()

        start_time = t.time()
        while True:
            remaining = delay - (t.time() - start_time)
            if remaining <= 0:
                break
            if self.stop_command_event.wait(
                timeout=remaining
            ) or self.terminate_event.wait(timeout=remaining):
                break
        if finish_event is not None:
            finish_event.set()

    def move(self, distance, **kwargs):
        speed = kwargs.get("speed", None)
        finish_event = kwargs.get("finish_event", None)

        command = Command(
            self._move_controlled, f"Moving {distance}m", distance, speed, finish_event
        )
        with self._queue_lock:
            print("I put a moving command to queue.")
            self.command_queue.put(command)

    def turn(self, angle, **kwargs):
        speed = kwargs.get("speed", None)
        finish_event = kwargs.get("finish_event", None)

        angle_deg = int(angle * 180 / pi)

        command = Command(
            self._turn_controlled, f"Turning {angle_deg}deg", angle, speed, finish_event
        )

        with self._queue_lock:
            print("Adding turning command to queue...")
            self.command_queue.put(command)

    def delay(self, delay, **kwargs):
        finish_event = kwargs.get("finish_event", None)

        command = Command(
            self._delay_controlled,
            f"Waiting {delay}s before next command.",
            delay,
            finish_event,
        )

        with self._queue_lock:
            self.command_queue.put(command)

    def get_speeds_motors(self):
        """Get current motor speeds."""
        return self.controller.get_motor_speed()

    def get_speed(self):
        return self.speed

    def turn_deg(self, angle, **kwargs):
        speed = kwargs.get("speed", None)
        finish_event = kwargs.get("finish_event", None)
        angle *= pi / 180
        self.turn(angle, speed=speed, finish_event=finish_event)

    def face(self, orientation, **kwargs):
        speed = kwargs.get("speed", None)
        finish_event = kwargs.get("finish_event", None)
        orientation %= 2 * pi
        angle = orientation - self.odometry.get_position()[2]
        self.turn(angle, speed=speed, finish_event=finish_event)

    def face_deg(self, orientation, **kwargs):
        speed = kwargs.get("speed", None)
        finish_event = kwargs.get("finish_event", None)
        self.face(
            orientation * pi / 180,
            speed=speed,
            finish_event=finish_event,
        )

    def move_centimeters(self, distance, **kwargs):
        speed = kwargs.get("speed", None)
        finish_event = kwargs.get("finish_event", None)
        distance /= 100
        self.move(distance, speed=speed, finish_event=finish_event)

    def get_position(self, centimeters=False):
        self.odometry.get_position(centimeters)

    def get_position_deg(self, centimeters=False):
        self.odometry.get_position_deg(centimeters)

    def reset_position(self, x=0, y=0, orientation=0):
        self.odometry.reset_position(x, y, orientation)

    def stop_command(self):
        self.stop_command_event.set()

    def clear_command_queue(self):
        with self._queue_lock:
            self.command_queue = queue.Queue()

    def terminate_controlled(self):
        print("Interrupting all the ongoing controlled movements...")
        self.terminate_event.set()
        self.stop_command_event.set()
        self.clear_command_queue()

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
                    self.turn_deg(angle)
                except ValueError:
                    print(f"Invalid angle value in instruction: {instruction}")
            elif instruction.startswith("f"):
                try:
                    orientation = float(instruction[1:])
                    self.face_deg(orientation)
                except ValueError:
                    print(f"Invalid orientation value in instruction: {instruction}")
            elif instruction.startswith("r"):
                try:
                    distance = float(instruction[1:])
                    self.move_centimeters(distance)
                except ValueError:
                    print(f"Invalid distance value in instruction: {instruction}")
                    print("Queued moving instruction...")
            elif instruction.startswith("d"):
                try:
                    delay = float(instruction[1:])
                    self.delay(delay)
                    print("Queued delay instruction...")
                except ValueError:
                    print(f"Invalid delay value in instruction: {instruction}")
            else:
                print(f"Unknown instruction: {instruction}")

    def shutdown(self):
        """Gracefully stop the worker thread and wait for it to finish."""
        print("Shutting down worker thread...")
        print("Waiting for all command to finish...")
        self.command_queue.join()
        print("All commands have been treated.")
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
        default=MotorConfig.DEFAULT_INSTR.value,
        help="Instruction string to execute",
    )
    args = parser.parse_args()

    odo = OdometryController(0, 0, 0)
    motor_controller = MotorController(odo)
    print("+++++Starting executing instructions, without event managment...+++++")
    instr = args.instructions
    # # log
    # print("Executing the following instructions: ", instr)
    motor_controller.execute_instructions(instr)
    motor_controller.shutdown()
    print(
        f"----Odometry result : {odo.x * 100}cm, {odo.y * 100}cm, {odo.orientation * 180 / pi}deg"
    )

    print(
        "+++++Starting executing a long instruction, stopping it (after 5sec) mid execution, then continuing other instructions...+++++"
    )
    instr = "r400, a45, a-45, r10"
    odo = OdometryController(0, 0, 0)
    motor_controller = MotorController(odo)
    motor_controller.execute_instructions(instr)
    t.sleep(4)
    motor_controller.stop_command()
    motor_controller.shutdown()
    print(
        f"----Odometry result : {odo.x * 100}cm, {odo.y * 100}cm, {odo.orientation * 180 / pi}deg (not handling stopping event in test mode)"
    )

    print(
        "+++++Will now test executing instructions with event managment (simulating group control)+++++"
    )

    finish_event = threading.Event()
    odo = OdometryController(0, 0, 0)
    motor_controller = MotorController(odo)
    instructions = ["a", "t", "a", "t"]
    for instr in instructions:
        if instr == "a":
            motor_controller.move(1, finish_event=finish_event)
        elif instr == "t":
            motor_controller.turn_deg(90, finish_event=finish_event)
        print("Waiting for last instruction to finish... (checking every 0.2sec)")
        while not finish_event.is_set():
            t.sleep(0.2)
        finish_event.clear()
        print("Done waiting, starting next")
    print(
        f"----Odometry result : {odo.x * 100}cm, {odo.y * 100}cm, {odo.orientation * 180 / pi}deg"
    )
