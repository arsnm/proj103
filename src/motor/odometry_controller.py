from src.config import RobotDimensions
from numpy import degrees, cos, sin, pi


class OdometryController:
    def __init__(self, start_x, start_y, start_orientation):
        self.x = start_x  # in m
        self.y = start_y  # in m
        self.error_ticks = (0, 0)
        self.orientation = start_orientation  # in rad, 0 is north, pi/2 is west
        self.orientation_deg = int(degrees(start_orientation))

    def update_position_from_ticks(self, ticks_left, ticks_right, small_error=False):
        if ticks_left * ticks_right < 0:  # opposite direction -> turned
            if ticks_left >= 0:  # turned left
                self.orientation += (
                    min(ticks_left, -ticks_right) / RobotDimensions.TICKS_PER_RAD
                )
                self.orientation %= 2 * pi
                self.orientation_deg = int(degrees(self.orientation))
                error_left, error_right = ticks_left - min(
                    ticks_left, -ticks_right
                ), ticks_right + min(ticks_left, -ticks_right)
            else:
                self.orientation -= (
                    min(-ticks_left, +ticks_right) / RobotDimensions.TICKS_PER_RAD
                )
                self.orientation %= 2 * pi
                self.orientation_deg = int(degrees(self.orientation))
                error_left, error_right = ticks_left + min(
                    -ticks_left, ticks_right
                ), ticks_right - min(-ticks_left, ticks_right)
        else:  # same direction -> moved
            if ticks_left < 0:  # moved backward
                direction = -1
            else:
                direction = 1
            ticks = min(abs(ticks_left), abs(ticks_right))
            self.x += direction * ticks * sin(self.orientation)
            self.y += direction * ticks * cos(self.orientation)
            error_left, error_right = (
                ticks_left - direction * ticks,
                ticks_right - direction * ticks,
            )

        if small_error:
            self.handle_small_error(error_left, error_right)
            self.error_ticks = (0, 0)
        else:
            self.error_ticks = (
                self.error_ticks[0] + error_left,
                self.error_ticks[1] + error_right,
            )

    def get_position(self):
        return (self.x, self.y, self.orientation_deg)

    def print_position(self):
        print(
            f"x: {self.x:.3f}, y: {self.y:.3f}, orientation:{self.orientation_deg:.2f}degC "
        )

    def get_position_centimeters(self):
        return (int(self.x * 100), int(self.y * 100), self.orientation_deg)

    def update_orientation_degrees(self):
        self.orientation_deg = int(degrees(self.orientation))

    def reset_position(self, x=0, y=0, orientation=0):
        self.x = x
        self.y = y
        self.orientation = orientation % (2 * pi)
        self.update_orientation_degrees()

    def handle_error(self):
        self.update_position_from_ticks(*self.error_ticks, True)

    def handle_small_error(
        self, error_left, error_right
    ):  # will be executed at the end of each movement
        # NOTE: one error should always be 0 if things are done properly
        if error_left == 0:  # turned and moved right
            circle_arc = error_right / RobotDimensions.TICKS_PER_METER
            delta_theta = circle_arc / RobotDimensions.WHEEL_BASE
            pivot_x = self.x + RobotDimensions.WHEEL_BASE * sin(self.orientation)
            pivot_y = self.y - RobotDimensions.WHEEL_BASE * cos(self.orientation)
            self.orientation += delta_theta
            self.orientation %= 2 * pi
        elif error_right == 0:  # turned and moved left
            circle_arc = error_left / RobotDimensions.TICKS_PER_METER
            delta_theta = circle_arc / RobotDimensions.WHEEL_BASE
            pivot_x = self.x - RobotDimensions.WHEEL_BASE * sin(self.orientation)
            pivot_y = self.y + RobotDimensions.WHEEL_BASE * cos(self.orientation)
            self.orientation -= delta_theta
        else:
            raise ValueError("ERROR - error in ticks was not properly handled")

        self.orientation %= 2 * pi
        self.x = (
            cos(delta_theta) * (self.x - pivot_x)
            - sin(delta_theta) * (self.y - pivot_y)
            + pivot_x
        )
        self.y = (
            sin(delta_theta) * (self.x - pivot_x)
            + cos(delta_theta) * (self.y - pivot_y)
            + pivot_y
        )


def main():
    odo = OdometryController(0, 0, 0)
    # testing full movement tracking
    ticks_left_list = [2002, -1503, -3008, 1400]
    ticks_right_list = [2005, 1488, -3008, -1405]
    for i in range(len(ticks_left_list)):
        odo.update_position_from_ticks(ticks_left_list[i], ticks_right_list[i], True)
        print(odo.get_position())
    odo.reset_position()

    # testing incremental movement tracking
    ticks_left_list = [1078, 1082, 1003, 89]
    ticks_right_list = [1080, 1080, 1000, 80]
    for i in range(len(ticks_left_list)):
        odo.update_position_from_ticks(ticks_left_list[i], ticks_right_list[i], False)
        print(odo.print_position())
    odo.handle_error()
    print(odo.print_position())
    odo.reset_position()


if __name__ == "__main__":
    main()
