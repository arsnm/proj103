from src.utils.grid_navigation import match_case_to_coord, match_coord_to_case
from src.motor.motor_controller import MotorController
from src.config import GridDimensions


class TargetController:
    def __init__(self, motor_controller):
        self.motor_controller = motor_controller
        self.running = False
        self.grid_case = GridDimensions.GRID_CASE
        self.grid_size = GridDimensions.GRID_SIZE
        self.motor_controller = motor_controller

    def target_case(self, start_x, start_y, case):
        self.running = True
        self.motor_controller.switch_mode("target")
        self.motor_controller.face_controlled(0)  # face north
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
        self.motor_controller.turn_deg(360)
        self.running = False

    def target_position(self, start_x, start_y, position):
        self.running = True
        self.motor_controller.switch_mode("target")
        self.motor_controller.face(0)
        self.motor_controller.move(position[1] - start_y)
        self.motor_controller.turn_deg(-90)  # face east
        self.motor_controller.move(position[0] - start_x)
        self.motor_controller.turn(0)
