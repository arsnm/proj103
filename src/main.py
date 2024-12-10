import time
import signal
import asyncio
from config import VisionConfig, ControlConfig, NetworkConfig, RobotConfig
from vision.camera import CameraManager
from vision.aruco_detector import ArucoDetector
from controllers.motor_controller import MotorController
from controllers.position_controller import PositionController
from controllers.race_controller import RaceController
from communication.websocket_manager import WebSocketManager
from communication.tracking_server_manager import TrackingServerManager
from utils.threading_utils import ThreadController
from models.position import Position
from models.robot_mode import RobotMode


class RobotSystem:
    def __init__(self, config_dict: dict):
        self.config = config_dict
        self.test_mode = config_dict["robot"].TEST_MODE

        # Initialize system state
        self.current_mode = RobotMode.MANUAL
        self.target_position = Position(0.0, 0.0, 0.0)

        # Initialize components
        self.camera = CameraManager(config_dict["vision"], self.test_mode)
        self.aruco_detector = ArucoDetector(
            config_dict["vision"].CALIBRATION_FILE, self.test_mode
        )
        self.cascaded_controller = MotorController(
            config_dict["control"], self.test_mode
        )
        self.position_controller = PositionController(config_dict["control"])
        self.race_controller = RaceController()
        self.ws_manager = WebSocketManager(config_dict["network"].WEBSOCKET_URI)
        self.tracking_server_manager = TrackingServerManager(
            config_dict["network"].TRACKING_SERVER_URL, self.race_controller
        )

        # Setup WebSocket callbacks
        self.setup_websocket_handlers()

        # Initialize thread controller
        self.thread_controller = ThreadController()
        self.setup_threads()

    def setup_websocket_handlers(self):
        """Setup handlers for WebSocket messages"""
        self.ws_manager.register_handler("mode_change", self.handle_mode_change)
        self.ws_manager.register_handler("target_position", self.handle_target_position)
        self.ws_manager.register_handler("manual_control", self.handle_manual_control)

    def handle_mode_change(self, data: dict):
        """Handle mode change request"""
        try:
            new_mode = RobotMode(data["mode"])
            self.current_mode = new_mode
            if new_mode == RobotMode.AUTOMATIC:
                # Reset position controller when entering automatic mode
                self.position_controller.set_target_pose(self.target_position)
        except ValueError as e:
            print(f"Invalid mode received: {e}")

    def handle_target_position(self, data: dict):
        """Handle new target position"""
        try:
            new_target = Position(
                x=float(data["x"]),
                y=float(data["y"]),
                theta=float(data.get("theta", 0.0)),
            )
            self.target_position = new_target
            if self.current_mode == RobotMode.AUTOMATIC:
                self.position_controller.set_target_pose(new_target)
        except (KeyError, ValueError) as e:
            print(f"Invalid target position data: {e}")

    def handle_manual_control(self, data: dict):
        """Handle manual control commands"""
        if self.current_mode != RobotMode.MANUAL:
            return

        try:
            left_speed = float(data["left"])
            right_speed = float(data["right"])
            self.cascaded_controller.set_target_velocities(left_speed, right_speed)
        except (KeyError, ValueError) as e:
            print(f"Invalid manual control data: {e}")

    def setup_threads(self):
        """Set up all control threads with their frequencies"""
        self.thread_controller.add_thread(
            "vision", self.vision_thread, self.config["vision"].CAMERA_FPS
        )

        self.thread_controller.add_thread(
            "position_control",
            self.position_control_thread,
            self.config["control"].POSITION_RATE,
        )

        self.thread_controller.add_thread(
            "motor_control",
            self.motor_control_thread,
            self.config["control"].MOTOR_RATE,
        )

        self.thread_controller.add_thread(
            "websocket", self.websocket_thread, self.config["network"].WEBSOCKET_RATE
        )

        self.thread_controller.add_thread(
            "tracking_server",
            self.tracking_server_thread,
            self.config["network"].TRACKING_RATE,
        )

    def vision_thread(self):
        """Process camera feed and estimate position"""
        ret, frame = self.camera.read_frame()
        if ret:
            pose = self.aruco_detector.estimate_robot_pose(frame)
            if pose:
                print("[VISION] - Updating robot position.")
                self.position_controller.update_position(pose)
                visible_flags = self.aruco_detector.get_visible_flags(frame)
                self.ws_manager.send_robot_status(
                    {
                        "pose": pose.__dict__,
                        "mode": self.current_mode.value,
                        "visible_markers": self.race_controller.new_flags(
                            visible_flags
                        )[0],
                    }
                )

            self.ws_manager.send_video_frame(frame)

    def position_control_thread(self):
        """High-level position control"""
        if self.current_mode == RobotMode.MANUAL:
            return

        current_pose = self.position_controller.get_current_pose()
        if current_pose:
            velocities = self.position_controller.compute_control(current_pose)
            if velocities:
                self.cascaded_controller.set_target_velocities(*velocities)

    def motor_control_thread(self):
        """Low-level motor control"""
        self.cascaded_controller.update_velocity_control()

    def websocket_thread(self):
        """Handle websocket communication"""
        if not self.ws_manager.connected:
            self.ws_manager.initialize_connection()

    def tracking_server_thread(self):
        """Handle communication to the tracking server"""
        self.tracking_server_manager.connect()
        asyncio.run(
            self.tracking_server_manager.update_position(
                self.position_controller.get_current_coord("cm")
            )
        )

    def start(self):
        """Start the robot system"""
        print("Initializing robot system...")
        print(f"Initial mode: {self.current_mode}")
        self.camera.initialize()
        self.thread_controller.start_all()
        print("Robot system started")

    def stop(self):
        """Stop the robot system"""
        print("Stopping robot system...")
        self.cascaded_controller.stop()
        self.thread_controller.stop_all()
        self.camera.release()
        print("Robot system stopped")


def main():
    # Create configuration
    config = {
        "vision": VisionConfig(),
        "control": ControlConfig(),
        "network": NetworkConfig(),
        "robot": RobotConfig(),
    }

    # Create robot system
    robot = RobotSystem(config)

    try:
        robot.start()

        # Keep main thread alive and monitor system
        while robot.thread_controller.is_running():
            time.sleep(1)
            status = robot.thread_controller.get_thread_status()
            print("System status:", status)

    except KeyboardInterrupt:
        print("\nShutdown requested...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        robot.stop()


if __name__ == "__main__":
    main()
