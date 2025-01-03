from pynput import keyboard
from src.motor.motor_controller import MotorController
from src.motor.odometry_controller import OdometryController


if __name__ == "__main__":
    odo = OdometryController(0, 0, 0)
    motor_controller = MotorController(odo)
    # Keep track of the currently pressed keys
    pressed_keys = set()

    # Callback function when a key is pressed
    def on_press(key):
        try:
            if key.char == "w" and "w" not in pressed_keys:
                pressed_keys.add("w")
                motor_controller.move_uncontrolled("forward")
            elif key.char == "s" and "s" not in pressed_keys:
                pressed_keys.add("s")
                motor_controller.move_uncontrolled("backward")
            elif key.char == "a" and "a" not in pressed_keys:
                pressed_keys.add("a")
                motor_controller.move_uncontrolled("left")
            elif key.char == "d" and "d" not in pressed_keys:
                pressed_keys.add("d")
                motor_controller.move_uncontrolled("right")
        except AttributeError:
            pass

    # Callback function when a key is released
    def on_release(key):
        try:
            if key.char == "w" and "w" in pressed_keys:
                pressed_keys.remove("w")
                motor_controller.move_uncontrolled("stop")
            elif key.char == "s" and "s" in pressed_keys:
                pressed_keys.remove("s")
                motor_controller.move_uncontrolled("stop")
            elif key.char == "a" and "a" in pressed_keys:
                pressed_keys.remove("a")
                motor_controller.move_uncontrolled("stop")
            elif key.char == "d" and "d" in pressed_keys:
                pressed_keys.remove("d")
                motor_controller.move_uncontrolled("stop")
        except AttributeError:
            pass

    # Set up the listener for keyboard events
    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
        listener.join()
