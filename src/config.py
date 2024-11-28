from dataclasses import dataclass


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

    # Low-level motor velocity control (encoder-based)
    MOTOR_RATE: int = 50  # 50 Hz
    MOTOR_PERIOD: float = 1 / MOTOR_RATE
    MOTOR_Kp: float = 0.8
    MOTOR_Ki: float = 0.3
    MOTOR_Kd: float = 0.1
    MOTOR_MAX_INTEGRAL: float = 1.0

    # I2C Configuration
    I2C_BUS: int = 8


@dataclass
class NetworkConfig:
    WEBSOCKET_URI: str = "ws://localhost:8765"
    MESSAGE_TIMEOUT: float = 0.1
    RECONNECT_DELAY: float = 1.0
    TRACKING_SERVER: str = "http://proj103.enst.fr/api"


@dataclass
class RobotConfig:
    GRID_SIZE: float = 2.0  # meters
    TEST_MODE: bool = False
