from numpy import pi


class NetworkConfig:
    WEBSOCKET_PORT = 8765
    HTTP_PORT = 8000
    TRACKING_SERVER_URL = "http://proj103.r2.enst.fr/api"
    TRACKING_SERVER_PORT = 80


class GridDimensions:
    GRID_CASE = 0.5  # size of a squared case in meter
    GRID_SIZE = (6, 7)  # dimension of the grid (taking the landing area into account)


class RobotDimensions:
    WHEEL_BASE = 0.18  # distance, in meters, between the two wheels
    WHEEL_RADIUS = 0.07  # radius, in meters, of a wheel
    TICKS_PER_ROT = 3800  # ticks for one full rotation of the wheel
    TICKS_PER_METER = TICKS_PER_ROT / (2 * pi * WHEEL_RADIUS)
    TICKS_PER_RAD = TICKS_PER_ROT * WHEEL_BASE / 2


class SpeedConfig:
    DEFAULT_MOVING_SPEED = 10
    DEFAULT_ROTATING_SPEED = 5
    DEFAULT_RAW_SPEED = 50
    MAX_SPEED = 40
    MIN_SPEED = 2  # useless
    MIN_RAW_SPEED = 5
    MAX_RAW_SPEED = 125


class RateConfig:
    ODOMETRY_FREQUENCY = 10  # Hz
    MOTOR_FREQUENCY = 100  # Hz


class PIDConfig:
    K_P = 0.5
    K_I = 0.5
    K_D = 0.5


class CalibrationConfig:
    pass
