# Main script running the whole thing

from src.control.automatic import AutomaticController
from src.control.target import TargetController
from src.motor.motor_controller import MotorController
from src.motor.odometry_controller import OdometryController
from src.vision.vision_controller import VisionController
from src.vision.camera_controller import CameraController
from src.networking.tracking_server import TrackingServerManager
from src.config import *

class RobotController:
    def __init__(self):
        self.current_mode = None
        self.camera_controller = CameraController()
        self.odometry_controller = OdometryController()
        self.motor_controller = MotorController(self.odometry_controller)
        self.vision_controller = VisionController(
            self.camera_controller, self.odometry_controller
        )
        self.vision_controller.start()
        self.interface_controller = InterfaceController()
        self.tracking_server_manager = TrackingServerManager(NetworkConfig.TRACKING_SERVER_URL)

    def main(self):


