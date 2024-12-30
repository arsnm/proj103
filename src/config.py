from dataclasses import dataclass
import numpy as np


@dataclass
class VisionConfig:
    CAMERA_FPS: int = 30
    FRAME_PERIOD: float = 1 / CAMERA_FPS
    CAMERA_WIDTH: int = 1280
    CAMERA_HEIGHT: int = 720
    CALIBRATION_FILE: str = "camera_calibration.npz"


@dataclass
class MotorConfig:
    SPEED_KP: float = 1.0
    SPEED_KI: float = 0.1
    SPEED_KD: float = 0.05
    MAX_INTEGRAL: float = 1000.0

    # Acceleration parameters (ticks/100ms/s)
    MAX_ACCELERATION: float = 40  # Maximum change in speed per second
    MAX_SPEED: int = 2000  # Maximum speed in ticks per 100ms

    UPDATE_RATE: int = 1000  # Hz


@dataclass
class ControlConfig:
    # High-level position control (camera-based)
    POSITION_RATE: int = 10  # Hz
    POSITION_PERIOD: float = 1 / POSITION_RATE
    POSITION_THRESHOLD_CM: float = 2.0
    ANGLE_THRESHOLD_RAD: float = 0.05
    MAX_SPEED_TICKS: int = 1000  # Maximum speed in ticks per 100ms

    # Robot parameters
    WHEEL_RADIUS_CM: float = 3.25
    WHEEL_BASE_CM: float = 17.0
    TICKS_PER_ROTATION: int = 3800

    # I2C Configuration
    I2C_BUS: int = 8

    # Grid Configuration
    GRID_SIZE_X: float = 300  # centimeters
    GRID_SIZE_Y: float = 350  # centimeters
    GRID_CASE_SIZE: float = 50  # centimeters

    # Initial position information
    INITIAL_X: int = 0
    INITIAL_Y: int = 0
    INITIAL_THETA: float = np.radians(0)

    @property
    def CM_PER_TICK(self) -> float:
        wheel_circumference = 2 * np.pi * self.WHEEL_RADIUS_CM
        return wheel_circumference / self.TICKS_PER_ROTATION

    @property
    def TICKS_PER_CM(self) -> float:
        wheel_circumference = 2 * np.pi * self.WHEEL_RADIUS_CM
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
