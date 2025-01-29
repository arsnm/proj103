from src.utils.grid_navigation import match_case_to_coord, match_coord_to_case
from src.motor.motor_controller import MotorController
from src.config import GridDimensions
import threading


class TargetController:
    def __init__(
        self,
        motor_controller,
        vision_controller,
        tracking_server,
    ):
        self.motor_controller = motor_controller
        self.vision_controller = vision_controller
        self.tracking_server = tracking_server
        self.motor_controller = motor_controller
        self.running = False
        self.finished_target = threading.Event()
        self._event_lock = threading.Lock()

    def run(self):
        self.running = True
        self.tracking_server.start()
        print("Lack the final implemenation")
        with self._event_lock:
            self.finished_target.set()

    def target(self, target, start=None, case_mode=False):
        if case_mode:
            self.target_case(target, start)
        else:
            self.target_position(target, start)

    def target_case(self, case, start):

        with self._event_lock:
            self.finished_target.clear()
        if start is None:
            start_x, start_y, start_orientation = self.motor_controller.get_position()
        else:
            start_x, start_y, start_orientation = start

        self.motor_controller.turn(-start_orientation)  # face north
        x_case, y_case = match_case_to_coord(case)
        start_case = match_coord_to_case(start_x, start_y)
        if case[1] == start_case[1]:
            pass
        else:
            self.motor_controller.move(y_case - start_y)
        self.motor_controller.turn_deg(-90)  # face east
        if case[0] == start_case[0]:
            pass
        else:
            self.motor_controller.move(x_case - start_x)
        self.motor_controller.turn_deg(360, finish_event=self.finished_target)
        # log
        print("Waiting for target to finish...")
        self.finished_target.wait()
        return self.vision_controller.get_flags()

    def target_position(self, target_position, start):

        with self._event_lock:
            self.finished_target.clear()
        if start is None:
            start_x, start_y, start_orientation = self.motor_controller.get_position()
        else:
            start_x, start_y, start_orientation = start

        self.motor_controller.turn(-start_orientation)
        self.motor_controller.move(target_position[1] - start_y)
        self.motor_controller.turn_deg(-90)  # face east
        self.motor_controller.move(target_position[0] - start_x)
        self.motor_controller.turn_deg(360, finish_event=self.finished_target)
        print("Waiting for target to finish...")
        self.finished_target.wait()
        return self.vision_controller.get_flags()

    def stop(self):
        if self.running:
            print("Stopping Target Controller...")
            self.finished_target.wait()
            self.tracking_server.stop()
            self.running = False
