from src.utils.pid_controller import PIDController
from src.config import PIDConfig, MotorConfig, RobotDimensions, RateConfig
from .odometry_controller import OdometryController
import time as t
import threading, queue, argparse
from numpy import pi

# import web_pdb

# debug
# web_pdb.set_trace(host="0.0.0.0", port=8080)


def init_pid(target: int = 0, min=-MotorConfig.MAX_SPEED, max=MotorConfig.MAX_SPEED):
    k_p = PIDConfig.K_P
    k_i = PIDConfig.K_I
    k_d = PIDConfig.K_D
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
        self.update_frequency = RateConfig.MOTOR_FREQUENCY
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
                command, args = item
                command(*args)
                try:
                    self.command_queue.task_done()
                except ValueError:
                    continue
            if self.terminate_event.is_set():
                print("Terminate event is set, finishing...")
                self.command_queue.shutdown(immediate=True)
                break

    def update_raw_speed(self, speed):
        try:
            if (
                type(speed) != int
                or speed < MotorConfig.MIN_RAW_SPEED
                or speed > MotorConfig.MAX_RAW_SPEED
            ):
                raise ValueError(
                    f"Raw speed should be a positive int between {MotorConfig.MIN_RAW_SPEED} and {MotorConfig.MAX_RAW_SPEED}, -{speed}- was provided"
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
                or speed < MotorConfig.MIN_SPEED
                or speed > MotorConfig.MAX_SPEED
            ):
                raise ValueError(
                    f"Speed should be a positive int between {MotorConfig.MIN_SPEED} and {MotorConfig.MAX_SPEED}, -{speed}- was provided"
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
            self.update_raw_speed(MotorConfig.DEFAULT_RAW_SPEED)
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

        # def update_controlled(self, target_ticks, direction: int, type):
        #     # type == True -> move
        #     # type == False --> turn
        #
        #     print("Updating controlled movements...")
        #     dt = 1 / self.update_frequency
        #     correction = 0
        #     odometry_rate = 1 / RateConfig.ODOMETRY_FREQUENCY
        #     next_odometry_update = t.time() + odometry_rate
        #     remaining_left, remaining_right = target_ticks
        #
        #     next_update = t.time() + dt
        #
        #     while not self.stop_event.is_set() and not self.terminate_event.is_set():
        #
        #         # NOTE: time between two updates must be higher than the time it takes to compute PID
        #
        #         overshoot_interval = int(
        #             2 * 100 * dt * (self.speed + abs(correction))
        #         )  # speed in ticks/0.01s
        #
        #         # log
        #         print(
        #             f"update_control at {t.time()} : {overshoot_interval}, {remaining_left}, {remaining_right}, {correction}"
        #         )
        #
        #         while (
        #             abs(remaining_left) < overshoot_interval
        #             or abs(remaining_right) < overshoot_interval
        #         ):
        #             self.speed //= 3
        #             overshoot_interval = int(2 * 100 * dt * (self.speed + abs(correction)))
        #         if self.speed <= 2:
        #             self.stop_event.set()
        #             break
        #
        #         speed_oriented = -direction * self.speed
        #         if type:
        #             print(
        #                 f"set speed : {speed_oriented + correction}, {speed_oriented - correction}"
        #             )
        #             self.controller.set_motor_speed(
        #                 speed_oriented + correction, speed_oriented - correction
        #             )
        #         else:
        #             self.controller.set_motor_speed(
        #                 -speed_oriented + correction, speed_oriented + correction
        #             )
        #
        #         t.sleep(max(next_update - t.time(), 0))
        #         next_update = t.time() + dt
        #
        #         ticks = self.controller.get_encoder_ticks()
        #         print(f"ticks:{ticks}")
        #
        #         remaining_left -= ticks[0]
        #         remaining_right -= ticks[1]
        #
        #         self.odometry_ticks = (
        #             self.odometry_ticks[0] + ticks[0],
        #             self.odometry_ticks[1] + ticks[1],
        #         )
        #         if t.time() > next_odometry_update:
        #             self.odometry.update_position_from_ticks(*self.odometry_ticks)
        #             self.odometry_ticks = (0, 0)
        #             next_odometry_update += odometry_rate
        #
        #         if type:
        #             error = (remaining_left - remaining_right) * 0.01 / dt
        #         else:
        #             error = (remaining_left + remaining_right) * 0.01 / dt
        #         print(f"Error: {error}")
        #
        #         try:
        #             if self.pid is not None:  # should always be true
        #                 correction = self.pid.compute(error, dt)
        #             else:
        #                 self.controller.standby()
        #                 raise ValueError("PID was not correctly initialized")
        #         except ValueError as e:
        #             print(f"ERROR - {e}")
        #
        #     self.controller.standby()
        #     ticks = self.controller.get_encoder_ticks()
        #     self.odometry.update_position_from_ticks(ticks[0], ticks[1], True)

    def move_controlled(self, distance: float, speed=None, no_wait=False):
        """Controlled movement with position feedback."""

        def command(distance, speed):
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
                self.update_speed(MotorConfig.DEFAULT_MOVING_SPEED)

            if self.test_mode:
                print(
                    f"TEST - Robot should move controlled at speed {self.speed} for {direction * distance}m."
                )
                return

            target_ticks = int(distance * RobotDimensions.TICKS_PER_METER)
            remaining_left = remaining_right = target_ticks

            motor_rate = 1 / RateConfig.MOTOR_FREQUENCY
            odometry_rate = 1 / RateConfig.ODOMETRY_FREQUENCY

            pid = init_pid(
                0,
                -MotorConfig.MAX_SPEED - self.speed,
                MotorConfig.MAX_SPEED - self.speed,
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

            t.sleep(0.5)

            return (remaining_left, remaining_right)

        item = (command, (distance, speed))
        if no_wait:
            self.add_command_to_front(item)
        else:
            self.command_queue.put(item)

    def turn_controlled(self, angle: float, speed=None, no_wait=False):
        """Controlled rotation with position feedback."""

        def command(angle, speed):
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
                self.update_speed(MotorConfig.DEFAULT_ROTATING_SPEED)

            if self.test_mode:
                print(
                    f"TEST - Robot should turn controlled {side * angle}rad at speed {self.speed}."
                )
                return
            target_ticks = int(angle * RobotDimensions.TICKS_PER_RAD)
            remaining_left = remaining_right = target_ticks

            motor_rate = 1 / RateConfig.MOTOR_FREQUENCY
            odometry_rate = 1 / RateConfig.ODOMETRY_FREQUENCY

            pid = init_pid(
                0,
                -MotorConfig.MAX_SPEED - self.speed,
                MotorConfig.MAX_SPEED - self.speed,
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

            t.sleep(0.5)

            return (remaining_left, remaining_right)

        item = (command, (angle, speed))
        if no_wait:
            self.add_command_to_front(item)
        else:
            self.command_queue.put(item)

    def get_speed(self):
        """Get current motor speeds."""
        return self.controller.get_motor_speed()

    def delay_controlled(self, delay, no_wait=False):
        """Add a delay between movements."""

        def command(delay):
            start_time = t.time()
            while True:
                remaining = delay - (t.time() - start_time)
                if remaining <= 0:
                    break
                if self.stop_event.wait(timeout=remaining) or self.terminate_event.wait(
                    timeout=remaining
                ):
                    break

        item = (command, (delay,))
        if no_wait:
            self.add_command_to_front(item)
        else:
            self.command_queue.put(item)

    def turn_controlled_deg(self, angle, speed=None, no_wait=False):
        angle *= pi / 180
        self.turn_controlled(angle, speed, no_wait)

    def face_controlled(self, orientation, speed=None, no_wait=False):
        orientation %= 2 * pi
        angle = orientation - self.odometry.get_position()[2]
        self.turn_controlled(angle, speed, no_wait)

    def face_controlled_deg(self, orientation, speed=None, no_wait=False):
        self.face_controlled(orientation * pi / 180, speed, no_wait)

    def move_controlled_centimeters(self, distance, speed=None, no_wait=False):
        distance /= 100
        self.move_controlled(distance, speed, no_wait)

    def terminate_controlled(self):
        print("Interrupting all the ongoing controlled movements...")
        self.terminate_event.set()
        self.stop_event.set()

    def clear_command_queue(self, immediate=False):
        if immediate:
            self.stop_event.set()
        self.command_queue.shutdown(True)
        self.command_queue = queue.Queue()
        self.stop_event.clear()

    def switch_mode(self, mode, immediate=False):
        if mode == self.current_mode:
            return
        if mode == "manual":
            self.clear_command_queue(True)

        def command(mode):
            self.current_mode = mode
            # TODO: Actually execute the right mode

        if immediate:
            self.clear_command_queue(True)
        else:
            self.command_queue.put((command, (mode,)))

    def add_command_to_front(self, command):
        """Add a command to the front of the queue."""
        with self.command_queue.mutex:
            self.command_queue.queue.appendleft(command)

    def execute_instructions(self, instructions):
        """Translate a list of instructions into movement executions."""
        instruction_list = instructions.split(",")
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

    odo = OdometryController(0, -0.25, 0)
    motor_controller = MotorController(odo)
    print("Starting executing instructions...")
    instr = args.instructions
    # # log
    # print("Executing the following instructions: ", instr)
    motor_controller.execute_instructions(instr)
    motor_controller.shutdown()
    print(odo.x * 100, odo.y * 100, odo.orientation * 180 / pi)
