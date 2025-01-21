from numpy import pi
from enum import Enum


class NetworkConfig(Enum):
    WEBSOCKET_PORT = 8765
    WEBSOCKET_URI = f"ws://robotpi-15.enst.fr:{WEBSOCKET_PORT}"
    HTTP_PORT = 8000
    TRACKING_SERVER_URL = "http://proj103.r2.enst.fr/api"
    TRACKING_SERVER_PORT = 80


class GridDimensions(Enum):
    GRID_CASE = 0.5  # size of a squared case in meter
    GRID_SIZE = (6, 7)  # dimension of the grid (taking the landing area into account)


class RobotDimensions(Enum):
    WHEEL_BASE = 0.173  # distance, in meters, between the two wheels
    WHEEL_RADIUS = 0.033  # radius, in meters, of a wheel
    TICKS_PER_ROT = 3800  # ticks for one full rotation of the wheel
    TICKS_PER_METER = TICKS_PER_ROT / (2 * pi * WHEEL_RADIUS)
    TICKS_PER_RAD = TICKS_PER_METER * WHEEL_BASE / 2


class MotorConfig(Enum):
    DEFAULT_MOVING_SPEED = 20
    DEFAULT_ROTATING_SPEED = 10
    DEFAULT_RAW_SPEED = 50
    MAX_SPEED = 50
    MIN_SPEED = 2
    MIN_RAW_SPEED = 5
    MAX_RAW_SPEED = 125
    DEFAULT_INSTR = "r20, a45, a-90, a45, r-20, a360"


class RateConfig(Enum):
    ODOMETRY_FREQUENCY = 2  # Hz
    MOTOR_FREQUENCY = 100  # Hz
    TRACKING_SERVER_FREQUENCY = 1  # Hz


class PIDConfig(Enum):
    K_P = 0.5
    K_I = 0.05
    K_D = 0.01


class CalibrationConfig(Enum):
    DIR_CAL_IMAGE = "calibration_images"
    CAL_FILE = "camera_calibration.npz"
    CHESS_WIDTH = 7
    CHESS_HEIGHT = 7
    CHESS_SIZE = 0.02  # in meters


class ArucoConfig(Enum):
    GRID_CASE = 0.5  # size of a squared case in meter
    ARUCO_DICT = "DICT_6X6_50"
    POSITION_SIZE = 0.1  # in meters
    FLAG_SIZE = 0.02  # in meters
    HINT_SIZE = 0.02
    DEFAULT_SIZE = 0.02
    FLAG_DIST_THRESHOLD = GRID_CASE * 0.7
    FLAG_ANGLE_THRESHOLD = 10 * pi / 180


class CameraConfig(Enum):
    WIDTH = 1280
    HEIGHT = 800
    FPS = 30


class PositionConfig(Enum):
    X_DIFF_THRESHOLD = 0.02
    Y_DIFF_THRESHOLD = 0.02
    ORIENTATION_DIFF_THRESHOLD = 5 * pi / 180


class StrategyConfig(Enum):
    NB_COLUMN_TO_CHECK = 1
    MAX_FLAG_TO_CAPTURE = 2
    RESPONSE_MOVEMENT = 202
    RESPONSE_SUCESS = 200
    MAX_RETRIES = 5
    ID = 15
    URL = "http://example.com"
    CHECK_INTERVAL = 1.0
    RETRY_DELAY = 5
