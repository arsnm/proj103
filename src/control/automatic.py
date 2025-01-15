from src.motor.motor_controller import MotorController
from src.config import GridDimensions, StrategyConfig
from threading import Event


class AutomaticController:
    def __init__(self, motor_controller):
        self.motor_controller = motor_controller
        self.running = False
        self.grid_case = GridDimensions.GRID_CASE
        self.grid_size = GridDimensions.GRID_SIZE
        self.nb_column_to_check = StrategyConfig.NB_COLUMN_TO_CHECK
        self.case_ongoing = Event()
        self.hint_detected = Event()

    def case_check(self):
        """Advance to next case and check both of its top corners. (assuming starting in the center of case)"""

        instructions = "r" + str(self.grid_case)
        instructions += "a45, a-90, a45"

        self.motor_controller.execute_instructions(instructions)

    def case_check_360(self):

        instructions = "r" + str(self.grid_case)
        instructions += "a45," * 4

        self.motor_controller.execute_instructions(instructions)

    def automatic(self, start_x, start_y, start_orientation, immediate=True):
        self.running = True
        self.motor_controller.switch_mode("automatic", immediate)
        for _ in range(self.nb_column_to_check):
            for _ in range(self.grid_size[1]):
                self.case_ongoing.wait()
                self.hint_detected.clear()
                self.motor_controller._automatic_case_started(self.case_ongoing)
                if self.hint_detected.is_set():
                    self.case_check_360()
                else:
                    self.case_check()
                self.motor_controller._automatic_case_finished()
        self.running = False

    def notify_hint(self):
        if self.running:
            self.hint_detected.wait()
            self.hint_detected.set()
