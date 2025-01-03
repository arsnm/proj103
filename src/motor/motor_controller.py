#!/usr/bin/python3

from src.utils.pid_controller import PIDController
from src.config import PIDConfig, SpeedConfig, RobotDimensions, RateConfig
from .odometry_controller import OdometryController
import time as t
import threading, queue, argparse
from numpy import pi

# import web_pdb

# debug
# web_pdb.set_trace(host="0.0.0.0", port=8080)


def init_pid(target: int = 0):
    k_p = PIDConfig.K_P
    k_i = PIDConfig.K_I
    k_d = PIDConfig.K_D
    return PIDController(
        k_p, k_i, k_d, target, SpeedConfig.MIN_SPEED, SpeedConfig.MAX_SPEED
    )


class MotorController:
    def __init__(
        self, odometry_controller: OdometryController, test_mode: bool = False
    ):
        self.test_mode = test_mode
        self.odometry = odometry_controller
        self.odometry_ticks = (0, 0)
        self.raw_speed: int = 50
        self.speed: int = 50
        self.pid = None
        self.error = (0, 0)
        self.update_frequency = RateConfig.MOTOR_FREQUENCY
        self.command_queue = queue.Queue()
        self.worker_thread = threading.Thread(
            target=self._command_processor, daemon=True
        )
        # log
        print("Should start worker_thread just after...")
        self.worker_thread.start()
        # log
        t.sleep(3)
        print("Did it even started ?!!")
        self.stop_event = threading.Event()
        self.terminate_event = threading.Event()
        self.action_lock = threading.Lock()

        if not test_mode:
            from .libMotors import controller as c

            self.controller = c.Controller()
            self.controller.set_motor_shutdown_timeout(5)
            self.controller.get_encoder_ticks()  # to init the ticks counter

    def _command_processor(self):
        # web_pdb.set_trace()
        # log
        print("Started _command_processor...")
        while True:
            command = self.command_queue.get()
            if self.terminate_event.is_set():
                # log
                print("Terminate event is set, finishing...")
                self.command_queue.task_done()
                break
            # log
            print("Processing command...")
            command()
            self.command_queue.task_done()
        # log
        print("Exiting _command_processor...")

    def _run_update_controlled(self, target_ticks, direction, type):
        # log
        print("Starting update thread...")
        update_thread = threading.Thread(
            target=self.update_controlled, args=(target_ticks, direction, type)
        )
        update_thread.start()
        update_thread.join()

    def update_raw_speed(self, speed):
        try:
            if (
                type(speed) != int
                or speed < SpeedConfig.MIN_RAW_SPEED
                or speed > SpeedConfig.MAX_RAW_SPEED
            ):
                raise ValueError(
                    f"Raw speed should be a positive int between {SpeedConfig.MIN_RAW_SPEED} and {SpeedConfig.MAX_RAW_SPEED}, -{speed}- was provided"
                )
            else:
                self.raw_speeds = speed
        except ValueError as e:
            print(f"ERROR - {e}")
        return

    def update_speed(self, speed):
        try:
            if (
                type(speed) != int
                or speed < SpeedConfig.MIN_SPEED
                or speed > SpeedConfig.MAX_SPEED
            ):
                raise ValueError(
                    f"Speed should be a positive int between {SpeedConfig.MIN_SPEED} and {SpeedConfig.MAX_SPEED}, -{speed}- was provided"
                )
            else:
                self.speeds = speed
        except ValueError as e:
            print(f"ERROR - {e}")
        return

    def move_uncontrolled(self, direction: str, speed=None):
        """Basic movement without control."""

        if speed is not None:
            self.update_raw_speed(speed)
        else:
            self.update_raw_speed(SpeedConfig.DEFAULT_RAW_SPEED)
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

    def update_controlled(self, target_ticks, direction: int, type):
        # type == True -> move
        # type == False --> turn

        print("Updating controlled movements...")
        dt = 1 / self.update_frequency
        next_update = t.time() + dt
        correction = 0
        odometry_rate = 1 / RateConfig.ODOMETRY_FREQUENCY
        next_odometry_update = t.time() + odometry_rate

        while not self.stop_event.is_set() and not self.terminate_event.is_set():

            # NOTE: time between two updates must be higher than the time it takes to compute PID

            overshoot_interval = int(2 * 100 * dt * self.speed)  # speed in ticks/0.01s
            remaining_left, remaining_right = target_ticks

            while (
                abs(remaining_left) < overshoot_interval
                or abs(remaining_right) < overshoot_interval
            ):
                self.speed //= 3
                overshoot_interval = int(2 * 100 * dt * self.speed)
            if self.speed <= 2:
                self.stop_event.set()
                break

            speed_oriented = -direction * self.speed
            if type:
                self.controller.set_motor_speed(
                    speed_oriented + correction, speed_oriented - correction
                )
            else:
                self.controller.set_motor_speed(
                    -speed_oriented + correction, speed_oriented + correction
                )

            t.sleep(max(next_update - t.time(), 0))
            next_update = t.time() + dt

            ticks = self.controller.get_encoder_ticks()

            self.odometry_ticks = (
                self.odometry_ticks[0] + ticks[0],
                self.odometry_ticks[1] + ticks[1],
            )
            if t.time() > next_odometry_update:
                self.odometry.update_position_from_ticks(*self.odometry_ticks)
                self.odometry_ticks = (0, 0)
                next_odometry_update += odometry_rate

            remaining_left -= ticks[0]
            remaining_right -= ticks[1]

            if type:
                error = (remaining_left - remaining_right) * 0.01 / dt
            else:
                error = (remaining_left + remaining_right) * 0.01 / dt

            try:
                if self.pid is not None:  # should always be true
                    correction = self.pid.compute(error, dt)
                else:
                    raise ValueError("PID was not correctly initialized")
            except ValueError as e:
                print(f"ERROR - {e}")

        self.controller.standby()
        ticks = self.controller.get_encoder_ticks()
        self.odometry.update_position_from_ticks(ticks[0], ticks[1], True)

    def move_controlled(self, distance, speed=None, no_wait=False):
        """Controlled movement with motor ticks feedback (PID)"""

        def command():
            if speed is not None:
                self.update_speed(speed)
            else:
                self.update_speed(SpeedConfig.DEFAULT_MOVING_SPEED)

            if self.test_mode:
                print(
                    f"TEST - Robot should move controlled at speed {self.speed} for {distance}m."
                )
                return

            print(f"MOTOR - Moving {distance}m at speed {self.speed}")

            target_ticks = int(distance * RobotDimensions.TICKS_PER_ROT)
            direction = -1 if distance < 0 else 1
            self.pid = init_pid()
            self._run_update_controlled((target_ticks, target_ticks), direction, True)

        if no_wait:
            self.add_command_to_front(command)
        else:
            # log
            print("Adding moving command to queue...")
            self.command_queue.put(command)

    def turn_controlled(self, angle, speed=None, no_wait=False):
        """Controlled turn with motor ticks feedback (PID)"""

        def command():
            if speed is not None:
                self.update_speed(speed)
            else:
                self.update_speed(SpeedConfig.DEFAULT_MOVING_SPEED)
            speed = -self.speed

            if self.test_mode:
                print(
                    f"TEST - Robot should turn controlled at speed {self.speed} for {angle * 180 / pi:.2f}deg"
                )
                return

            print(f"MOTOR - Turning {angle} at speed {self.speed}")

            angle = angle % (2 * pi) - pi / 2
            target_ticks = int(angle * RobotDimensions.TICKS_PER_RAD)
            direction = -1 if angle < 0 else 1
            self.pid = init_pid()
            self._run_update_controlled((target_ticks, -target_ticks), direction, False)

        if no_wait:
            self.add_command_to_front(command)
        else:
            self.command_queue.put(command)

    def delay_controlled(self, delay):
        """Add a delay between movements."""

        def command():
            t.sleep(delay)

        self.command_queue.put(command)

    def turn_controlled_deg(self, angle, speed=None):
        angle *= pi / 180
        self.turn_controlled(angle, speed)

    def move_uncontrolled_centimeters(self, distance, speed=None):
        distance /= 100
        self.move_controlled(distance, speed)

    def stop_all_controlled(self):
        print("Interrupting all the ongoing controlled movements...")
        self.terminate_event.set()
        self.stop_event.set()
        with self.command_queue.mutex:
            self.command_queue.queue.clear()

    def add_command_to_front(self, command):
        """Add a command to the front of the queue."""
        with self.command_queue.mutex:
            self.command_queue.queue.appendleft(command)

    def execute_instructions(self, instructions):
        """Translate a list of instructions into movement executions."""
        instruction_list = instructions.split(",")
        # log
        print(instruction_list)
        for instruction in instruction_list:
            instruction = instruction.strip()
            if instruction.startswith("a"):
                try:
                    angle = float(instruction[1:])
                    self.turn_controlled_deg(angle)
                    print("Queued turning instruction...")
                except ValueError:
                    print(f"Invalid angle value in instruction: {instruction}")
            elif instruction.startswith("r"):
                try:
                    distance = float(instruction[1:])
                    # log
                    print("Queuing moveing instaurction of ", distance, "cm")
                    self.move_uncontrolled_centimeters(distance)
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


if __name__ == "__main__":
    # parser.add_argument("-i", "--instructions", type=str, help="Instruction string to execute")
    # parser = argparse.ArgumentParser(description="Motor Controller CLI")
    # args = parser.parse_args()

    odo = OdometryController(0, -25, 0)
    motor_controller = MotorController(odo)
    print("Starting executing instructions...")
    instr = "r10"
    # log
    print("Executing the following instructions: ", instr)
    motor_controller.execute_instructions(instr)
