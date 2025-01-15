# Main script running the whole thing

from src.control.automatic import AutomaticController
from src.control.target import TargetController
from src.motor.motor_controller import MotorController
from src.motor.odometry_controller import OdometryController
from src.vision.vision_controller import VisionController
from src.vision.camera_controller import CameraController
from src.networking.tracking_server import TrackingServerManager
from src.networking.websocket_manager import WebSocketManager
from src.web.server import CombinedServer
from src.config import *


class RobotController:
    def __init__(self):
        self.current_mode = None
        self.current_mode = "manual"
        self.camera_controller = CameraController()
        self.odometry_controller = OdometryController()
        self.motor_controller = MotorController(self.odometry_controller)
        self.automatic_controller = AutomaticController(self.motor_controller)
        self.target_controller = TargetController(self.motor_controller)
        self.vision_controller = VisionController(
            self.camera_controller,
            self.odometry_controller,
            self.automatic_controller,
            self.target_controller,
        )
        self.websocket_manager = WebSocketManager(NetworkConfig.WEBSOCKET_URI)
        self.tracking_server_manager = TrackingServerManager(
            NetworkConfig.TRACKING_SERVER_URL
        )
        self.server = CombinedServer()
        # self.race_controller = race_controller()

    async def start(self):
        print("Starting Robot System...")
        self.vision_controller.start()
        await self.server.start()
        self.websocket_manager.initialize_connection()

    def setup_websocket_handlers(self):
        """Setup handlers for WebSocket messages"""
        self.websocket_manager.register_handler("mode_change", self.handle_mode_change)
        self.websocket_manager.register_handler(
            "target_position", self.handle_target_position
        )
        self.websocket_manager.register_handler(
            "manual_control", self.handle_manual_control
        )

    def handle_mode_change(self, data: dict):
        """Handle mode change request"""
        new_mode = data["mode"]
        self.current_mode = new_mode
        if new_mode == "automatic":
            # Reset position controller when entering automatic mode
            self.position_controller.set_target_pose(self.target_position)
        elif new_mode == "manual":
            pass
        elif new_mode == "target":
            pass
        else:
            print(f"Invalid mode received: {new_mode}")

    def handle_target_position(self, data: dict):
        """Handle new target position"""
        try:
            new_target = float(data["x"]), float(data["y"])
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

    def tracking_server_thread(self):
        """Handle communication to the tracking server"""
        self.tracking_server_manager.connect()
        asyncio.run(
            self.tracking_server_manager.update_position(
                self.position_controller.get_current_coord("cm")
            )
        )

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
