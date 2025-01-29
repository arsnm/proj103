import curses
from src.motor.motor_controller import MotorController


class ManualController:
    def __init__(self, motor_controller):
        self.motor_controller = motor_controller
        self.running = False

    def run(self):
        if not self.running:
            self.running = True
            self.motor_controller.clear_queue()
        self.motor_controller.update_timeout(10)

    def execute(self, direction, speed=None):
        if not self.running:
            print(
                "ERROR - Cannot execute manual control if manual controller is not running."
            )
            return
        self.motor_controller.move_uncontrolled(direction, speed)

    def stop(self):
        if self.running:
            self.motor_controller.update_timeout(1)
            self.running = False


def main(stdscr):
    from src.motor.odometry_controller import OdometryController

    # Clear screen
    stdscr.clear()

    odo = OdometryController(0, 0, 0)
    motor_controller = MotorController(odo)
    pressed_keys = set()

    # Instructions for the user
    stdscr.addstr(0, 0, "Use W, A, S, D to control the robot. Press Q to quit.")
    stdscr.refresh()

    # Set timeout for getch() to 100ms
    stdscr.timeout(100)

    while True:
        key = stdscr.getch()

        if key == ord("q"):
            break
        elif key == ord("w"):
            if "w" not in pressed_keys:
                pressed_keys.add("w")
                motor_controller.move_uncontrolled("forward")
        elif key == ord("s"):
            if "s" not in pressed_keys:
                pressed_keys.add("s")
                motor_controller.move_uncontrolled("backward")
        elif key == ord("a"):
            if "a" not in pressed_keys:
                pressed_keys.add("a")
                motor_controller.move_uncontrolled("left")
        elif key == ord("d"):
            if "d" not in pressed_keys:
                pressed_keys.add("d")
                motor_controller.move_uncontrolled("right")

        # Check for key releases
        if key == -1:
            if "w" in pressed_keys:
                pressed_keys.remove("w")
                motor_controller.move_uncontrolled("stop")
            if "s" in pressed_keys:
                pressed_keys.remove("s")
                motor_controller.move_uncontrolled("stop")
            if "a" in pressed_keys:
                pressed_keys.remove("a")
                motor_controller.move_uncontrolled("stop")
            if "d" in pressed_keys:
                pressed_keys.remove("d")
                motor_controller.move_uncontrolled("stop")


if __name__ == "__main__":
    curses.wrapper(main)
