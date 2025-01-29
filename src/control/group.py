from src.vision.camera_controller import CameraController
from src.motor.motor_controller import MotorController
from src.motor.odometry_controller import OdometryController
from src.networking.tracking_server import TrackingServerManager
from src.vision.vision_controller import VisionController
import threading
import requests
import time
import json
from src.config import StrategyConfig


class GroupController:
    def __init__(
        self,
        motor_controller,
        vision_controller,
        tracking_server,
        global_status=None,
        server_url=StrategyConfig.URL.value,
        id=StrategyConfig.ID.value,
        check_interval=StrategyConfig.CHECK_INTERVAL.value,
        max_retries=StrategyConfig.MAX_RETRIES.value,
        retry_delay=StrategyConfig.RETRY_DELAY.value,
    ):
        self.motor_controller: MotorController = motor_controller
        self.vision_controller: VisionController = vision_controller
        self.tracking_server: TrackingServerManager = tracking_server
        self.global_status = global_status
        self.server_url = server_url
        self.id = id
        self.check_interval = check_interval
        self.max_retries = max_retries
        self.retry_delay = retry_delay
        self.running = False
        self.monitor_thread = None
        self.consecutive_failures = 0
        self._finished_movement = threading.Event()

    def run(self):
        """Start the group strategy thread."""
        if not self.running:
            self.running = True
            self.motor_controller.clear_command_queue()
            self.thread = threading.Thread(target=self._monitoring_loop)
            self.thread.daemon = True
            self.thread.start()
            # self.tracking_server.start()
            print(f"Started group strategy {self.server_url}")
        else:
            print("Monitoring is already running")

    def execute_instructions(self, json_data):
        print(f"Executing instruction {json_data}...")
        if json_data[0] == StrategyConfig.MOVE_MESSAGE:
            self.motor_controller.move_centimeters(
                json_data[1] if json_data[1] is not None else 50,
                finish_event=self._finished_movement,
            )
        elif json_data[0] == StrategyConfig.TURN_MESSAGE:
            self.motor_controller.turn_deg(
                json_data[1] if json_data[1] is not None else 45,
                finish_event=self._finished_movement,
            )
        elif json_data[0] == StrategyConfig.CAPTURE_MESSAGE:
            pose = self.motor_controller.get_position()
            self.tracking_server.send_marker(json_data[1], pose)
            self.motor_controller.turn_deg(360, finish_event=self._finished_movement)
        elif json_data[0] == StrategyConfig.REST_MESSAGE:
            self._finished_movement.set()
        self._finished_movement.wait()
        self._finished_movement.clear()

    def stop_movement(self):
        self.motor_controller.stop_command()

    def stop(self):
        """Stop the monitoring thread."""
        self.running = False
        # self.tracking_server.stop()
        self.motor_controller.clear_command_queue()
        if self.monitor_thread:
            self.monitor_thread.join()
            self.monitor_thread = None
            print("Stopped monitoring")

    def _monitoring_loop(self):
        """Main monitoring loop that sends periodic checks."""
        while self.running:
            try:
                self._send_check()
                time.sleep(self.check_interval)
            except Exception as e:
                print(f"Unexpected error in monitoring loop: {str(e)}")
                time.sleep(self.check_interval)

    def _send_check(self):
        response = requests.post(f"{self.server_url}/api/check?id={self.id}")

        print(f"response from server : {response.status_code}")

        if response.status_code == StrategyConfig.RESPONSE_MOVEMENT.value:
            try:
                json_data = response.json()
                print(f"json_data, received = {json_data}")
                if isinstance(json_data, list) and len(json_data):
                    self.consecutive_failures = 0
                    self.execute_instructions(json_data)
                else:
                    print(f"Unexpected JSON format: {json_data}")
                    return
            except json.JSONDecodeError:
                print("ERROR - Failed to decode JSON response")
                return
        else:
            print(f"Server returned status code {response.status_code}")

    # def _send_check(self):
    #     """Send a single check request to the server with retry logic."""
    #     retry_count = 0
    #
    #     headers = {"id": str(self.id - 1)}
    #
    #     while retry_count < self.max_retries:
    #         try:
    #             start_time = time.time()
    #             response = requests.get(self.server_url, headers=headers, timeout=5)
    #             response_time = (
    #                 time.time() - start_time
    #             ) * 1000  # Convert to milliseconds
    #
    #             if response.status_code == StrategyConfig.RESPONSE_SUCESS.value:
    #                 if self.consecutive_failures > 0:
    #                     print(
    #                         f"Connection restored after {self.consecutive_failures} failures!"
    #                     )
    #                 self.consecutive_failures = 0
    #                 # Only print every 10 successful checks to reduce output
    #                 if int(time.time()) % 10 == 0:
    #                     print(f"Server is UP - Response time: {response_time:.2f}ms")
    #                 return
    #
    #             elif response.status_code == StrategyConfig.RESPONSE_MOVEMENT.value:
    #                 try:
    #                     json_data = response.json()
    #                     if isinstance(json_data, list) and len(json_data):
    #                         self.consecutive_failures = 0
    #                         print(json_data)
    #                         self.execute_instructions(json_data)
    #                     else:
    #                         print(f"Unexpected JSON format: {json_data}")
    #                         return
    #                 except json.JSONDecodeError:
    #                     print("ERROR - Failed to decode JSON response")
    #                     return
    #             else:
    #                 print(f"Server returned status code {response.status_code}")
    #
    #         except requests.exceptions.ConnectionError:
    #             self.consecutive_failures += 1
    #             retry_count += 1
    #
    #             if retry_count < self.max_retries:
    #                 print(
    #                     f"ERROR - Connection failed - Retrying in {self.retry_delay} seconds... (Attempt {retry_count + 1}/{self.max_retries})"
    #                 )
    #                 time.sleep(self.retry_delay)
    #             else:
    #                 print(
    #                     f"ERROR - Connection failed after {self.max_retries} attempts - Server might be down"
    #                 )
    #
    #         except requests.exceptions.Timeout:
    #             self.consecutive_failures += 1
    #             retry_count += 1
    #
    #             if retry_count < self.max_retries:
    #                 print(
    #                     f"Request timed out - Retrying in {self.retry_delay} seconds... (Attempt {retry_count + 1}/{self.max_retries})"
    #                 )
    #                 time.sleep(self.retry_delay)
    #             else:
    #                 print(
    #                     f"ERROR - Request timed out after {self.max_retries} attempts"
    #                 )
    #
    #         except Exception as e:
    #             print(f"ERROR - Unexpected error: {str(e)}")
    #             return


# Example usage
if __name__ == "__main__":
    # Create a monitor instance with custom retry settings
    odo = OdometryController()
    motor_controller = MotorController(odo)
    camera_controller = CameraController()
    vision_controller = VisionController(camera_controller)
    tracking_server = TrackingServerManager(
        "http://proj103.r2.enst.fr", motor_controller
    )
    group_controller = GroupController(
        motor_controller,
        vision_controller,
        tracking_server,
        None,
        "http://137.194.13.177:8080",
        check_interval=1.0,
        max_retries=3,
        retry_delay=5,
    )

    try:
        # Start monitoring
        group_controller.run()

        # Keep the main thread running
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        # Handle graceful shutdown on Ctrl+C
        group_controller.stop()
        print("\nMonitoring stopped")
