from src.utils.grid_navigation import match_case_to_coord, match_coord_to_case
from src.motor.motor_controller import MotorController


class TargetController:
    def __init__(self, motor_controller, grid_case, grid_size):
        self.motor_controller = motor_controller
        self.running = False
        self.grid_case = grid_case
        self.grid_size = grid_size
        self.motor_controller = motor_controller
        self.running = False

    def target(self, start_x, start_y, case):
        self.motor_controller.switch_mode("target")
        self.motor_controller.face_controlled(0)  # face north
        x_case, y_case = match_case_to_coord(case)
        start_case = match_coord_to_case(start_x, start_y)
        if case[1] == start_case[1]:
            pass
        else:
            self.motor_controller.move_controlled(y_case - start_y)
        self.motor_controller.turn_controlled_deg(-90)  # face east
        if case[0] == start_case[0]:
            pass
        else:
            self.motor_controller.move_controlled(x_case - start_x)
        self.motor_controller.turn_controlled_deg(360)
