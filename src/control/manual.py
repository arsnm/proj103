import curses
from src.motor.motor_controller import MotorController
from src.motor.odometry_controller import OdometryController

def main(stdscr):
    # Clear screen
    stdscr.clear()

    odo = OdometryController(0, 0, 0)
    motor_controller = MotorController(odo)
    pressed_keys = set()

    # Instructions for the user
    stdscr.addstr(0, 0, "Use W, A, S, D to control the robot. Press Q to quit.")
    stdscr.refresh()

    while True:
        key = stdscr.getch()

        if key == ord('q'):
            break
        elif key == ord('w'):
            if 'w' not in pressed_keys:
                pressed_keys.add('w')
                motor_controller.move_uncontrolled("forward")
        elif key == ord('s'):
            if 's' not in pressed_keys:
                pressed_keys.add('s')
                motor_controller.move_uncontrolled("backward")
        elif key == ord('a'):
            if 'a' not in pressed_keys:
                pressed_keys.add('a')
                motor_controller.move_uncontrolled("left")
        elif key == ord('d'):
            if 'd' not in pressed_keys:
                pressed_keys.add('d')
                motor_controller.move_uncontrolled("right")
        elif key == curses.KEY_UP:
            if 'w' in pressed_keys:
                pressed_keys.remove('w')
                motor_controller.move_uncontrolled("stop")
        elif key == curses.KEY_DOWN:
            if 's' in pressed_keys:
                pressed_keys.remove('s')
                motor_controller.move_uncontrolled("stop")
        elif key == curses.KEY_LEFT:
            if 'a' in pressed_keys:
                pressed_keys.remove('a')
                motor_controller.move_uncontrolled("stop")
        elif key == curses.KEY_RIGHT:
            if 'd' in pressed_keys:
                pressed_keys.remove('d')
                motor_controller.move_uncontrolled("stop")

if __name__ == "__main__":
    curses.wrapper(main)