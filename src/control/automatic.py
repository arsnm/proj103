from src.motor.motor_controller import MotorController
from src.config import GridDimensions


class AutomaticController:
    def __init__(self, motor_controller, grid_case, grid_size, nb_column_to_check):
        self.motor_controller = motor_controller
        self.running = False
        self.grid_case = grid_case
        self.grid_size = grid_size
        self.nb_column_to_check = nb_column_to_check

    def case_check(self):
        """Advance to next case and check both of its top corners. (assuming starting in the center of case)"""

        instructions = "r" + str(self.grid_case)
        instructions += "a45, a-90, a45"

        self.motor_controller.execute_instructions(instructions)

    def automatic(self, start_x, start_y, start_orientation, immediate):
        for _ in range(self.nb_column_to_check):
            for _ in range(self.grid_size[1]):
                self.case_check()
