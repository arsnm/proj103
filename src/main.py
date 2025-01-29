# Main script running the whole thing

import threading
import time
from src.control.automatic import AutomaticController
from src.control.target import TargetController
from src.control.manual import ManualController
from src.control.group import GroupController
from src.motor.motor_controller import MotorController
from src.motor.odometry_controller import OdometryController
from src.vision.vision_controller import VisionController
from src.vision.camera_controller import CameraController
from src.networking.tracking_server import TrackingServerManager
from src.networking.websocket_manager import WebSocketManager
from src.web.server import CombinedServer
from src.config import *

# Variables
vue_app_dir = NetworkConfig.VUE_APP_DIR.value
video_stream_dir = NetworkConfig.VIDEO_HLS_DIR.value
tracking_url = NetworkConfig.TRACKING_SERVER_URL.value


class RobotController:
    def __init__(self):
        self.camera_controller = CameraController()

        # Robot's function controllers
        self.vision_controller = VisionController(self.camera_controller)
        self.odometry_controller = OdometryController()
        self.motor_controller = MotorController(self.odometry_controller)

        # Networking
        self.tracking_server = TrackingServerManager(
            NetworkConfig.TRACKING_SERVER_URL.value
        )
        self.websocket_manager = WebSocketManager(NetworkConfig.WEBSOCKET_URI.value)
        self.update_status_thread = threading.Thread(target=self._update_status_loop)

        # Control Controllers
        self.automatic_controller = AutomaticController(
            self.motor_controller, self.vision_controller, self.tracking_server
        )
        self.target_controller = TargetController(
            self.motor_controller, self.vision_controller, self.tracking_server
        )
        self.manual_controller = ManualController(self.motor_controller)
        self.group_controller = GroupController(
            self.motor_controller, self.vision_controller, self.tracking_server
        )

        # Robot status
        self.vision_pos = None
        self.odometry_pos = None
        self.speed = None
        self.mode = "manual"
        self.running = False

    def _update_status_loop(self):
        while self.running:
            self.vision_pos = self.vision_controller.get_position()
            self.odometry_pos = self.odometry_controller.get_position()
            self.speed = self.motor_controller.get_speed()

            if self.websocket_manager.connected:
                # log
                print("TODO: Should send status to web interface(s)")
                self.websocket_manager.send_robot_status(
                    {
                        "position": self.odometry_pos,
                        "speed": self.speed,
                        "mode": self.mode,
                    }
                )
                time.sleep(NetworkConfig.UPDATE_ROBOT_STATUS_RATE.value)

    def start(self):
        print("Starting Robot Controller...")
        self.vision_controller.start()
        self.websocket_manager.initialize_connection()
        self.update_status_thread.start()
        self.running = True

    def setup_websocket_handlers(self):
        """Setup handlers for WebSocket messages"""
        self.websocket_manager.register_handler("mode_change", self.handle_mode_change)
        self.websocket_manager.register_handler(
            "manual_control", self.handle_manual_control
        )

    def handle_mode_change(self, data: dict, init=False):
        """Handle mode change request"""
        old_mode = self.mode
        new_mode = data["mode"]

        if old_mode == new_mode and not init:
            return

        if old_mode == "automatic":
            self.automatic_controller.stop()
        elif old_mode == "manual":
            self.manual_controller.stop()
        elif old_mode == "target":
            self.target_controller.stop()
        elif old_mode == "group":
            self.target_controller.stop()

        self.mode = new_mode
        if new_mode == "automatic":
            start = data.get("start", None)
            self.automatic_controller.run()
            self.automatic_controller.automatic(start[0], start[1], start[2])
        elif new_mode == "manual":
            self.manual_controller.run()
        elif new_mode == "target":
            start = data.get("start", None)
            target_coord = data.get("target_coord", None)
            target_case = data.get("target_case", None)
            self.target_controller.run()
            if target_case is not None:
                self.target_controller.target(target_case, start, True)
            elif target_coord is not None:
                self.target_controller.target(target_coord, start)
        elif new_mode == "group":
            self.group_controller.run()
        else:
            print(f"Invalid mode received: {new_mode}")

    def handle_manual_control(self, data: dict):
        """Handle manual control commands"""
        if self.mode != "manual":
            return
        try:
            direction = float(data["direction"])
            speed = float(data["speed"])
            self.manual_controller.execute(direction, speed)
        except (KeyError, ValueError) as e:
            print(f"Invalid manual control data: {e}")

    def stop(self):
        """Stop the robot system"""
        print("Stopping robot system...")
        self.motor_controller.shutdown()
        self.vision_controller.stop()
        self.automatic_controller.stop()
        self.target_controller.stop()
        self.manual_controller.stop()
        self.running = False
        self.update_status_thread.join()
        print("Robot system stopped.")


class RobotSystem:
    def __init__(self):
        self.server = CombinedServer(video_stream_dir, vue_app_dir)
        self.robot_controller = RobotController()
        self.running = False

    def start(self):
        self.running = True
        self.server.start()
        self.robot_controller.start()

    def stop(self):
        self.server.stop()
        self.robot_controller.stop()
        self.running = False


def main():
    import time

    # Create robot system
    robot_system = RobotSystem()

    try:
        robot_system.start()
        # Keep main thread alive and monitor system
        while robot_system.running:
            time.sleep(1)
        time.sleep(1)
    except KeyboardInterrupt:
        print("\nShutdown requested...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        robot_system.stop()


if __name__ == "__main__":
    main()
