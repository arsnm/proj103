from dataclasses import dataclass
import numpy as np


@dataclass
class VisionConfig:
    CAMERA_FPS: int = 30
    FRAME_PERIOD: float = 1 / CAMERA_FPS
    CAMERA_WIDTH: int = 640
    CAMERA_HEIGHT: int = 480
    CALIBRATION_FILE: str = "camera_calibration.npz"


@dataclass
class ControlConfig:
    # High-level position control (camera-based)
    POSITION_RATE: int = 10  # 10 Hz
    POSITION_PERIOD: float = 1 / POSITION_RATE
    POSITION_Kp: float = 0.5
    POSITION_Ki: float = 0.1
    POSITION_Kd: float = 0.1
    POSITION_MAX_INTEGRAL: float = 1.0

    # Low-level odometry control (encoder-based)
    ODOMETRY_RATE: int = 50  # 50 Hz
    WHEEL_RADIUS: float = 6.9  # in cm
    WHEEL_BASE: float = 6.9  # in cm
    TICKS_PER_ROTATION: int = 3800

    # Speed limit
    MAX_TICKS_PER_SEC = 1000

    # Low-level motor velocity control (encoder-based)
    MOTOR_RATE: int = 100  # 50 Hz
    MOTOR_PERIOD: float = 1 / MOTOR_RATE
    MOTOR_Kp: float = 0.8
    MOTOR_Ki: float = 0.3
    MOTOR_Kd: float = 0.1
    MOTOR_MAX_INTEGRAL: float = 1.0

    # I2C Configuration
    I2C_BUS: int = 8

    # Grid Configuration
    GRID_SIZE_X: float = 300  # centimeters
    GRID_SIZE_Y: float = 350  # centimeters
    CASE_SIZE: float = 50  # centimeters

    @property
    def CM_PER_TICK(self) -> float:
        wheel_circumference = 2 * np.pi * self.WHEEL_RADIUS
        return wheel_circumference / self.TICKS_PER_ROTATION

    @property
    def TICKS_PER_CM(self) -> float:
        wheel_circumference = 2 * np.pi * self.WHEEL_RADIUS
        return self.TICKS_PER_ROTATION / wheel_circumference


@dataclass
class TrackingServerConfig:
    API_RESPONSE = {
        200: "200 - Request fully treated",
        400: "400 - Error in request",
        401: "401 - Authentification error",
        404: "404 - API command not recognized",
        500: "500 - Internal error of the server",
        503: "503 - Valid request, but race not started",
    }
    TEAM_ID = 15


@dataclass
class NetworkConfig:
    WEBSOCKET_URI: str = "ws://localhost:8765"
    MESSAGE_TIMEOUT: float = 0.1
    RECONNECT_DELAY: float = 1.0
    TRACKING_SERVER_URL: str = "http://proj103.enst.fr/api"
    TRACKING_RATE: int = 1
    WEBSOCKET_RATE: int = 30
    TRACKING_SERVER_CONFIG = TrackingServerConfig()


@dataclass
class RobotConfig:
    TEST_MODE: bool = False
