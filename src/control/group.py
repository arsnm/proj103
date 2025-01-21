from src.networking.group_server import GroupServerManager
from src.motor.motor_controller import MotorController
import threading
import requests
import time
from datetime import datetime
import json
from src.config import StrategyConfig


class GroupServerManager:
    def __init__(
        self,
        motor_controller,
        tracking_server,
        server_url=StrategyConfig.URL,
        id=StrategyConfig.ID,
        check_interval=StrategyConfig.CHECK_INTERVAL,
        max_retries=StrategyConfig.MAX_RETRIES,
        retry_delay=StrategyConfig.RETRY_DELAY,
    ):
        """
        Initialize the HTTP server monitor.

        Args:
            server_url (str): The URL of the server to monitor
            check_interval (float): Time between checks in seconds (default: 1.0)
            max_retries (int): Maximum number of retry attempts when connection fails
            retry_delay (float): Time to wait between retries in seconds
        """
        self.motor_controller = motor_controller
        self.tracking_server = tracking_server
        self.server_url = server_url.value
        self.id = id.value
        self.check_interval = check_interval.value
        self.max_retries = max_retries.value
        self.retry_delay = retry_delay.value
        self.is_running = False
        self.monitor_thread = None
        self.consecutive_failures = 0
        self._ongoing_movement = threading.Event()

    def start_monitoring(self):
        """Start the monitoring thread."""
        if not self.is_running:
            self.is_running = True
            self.monitor_thread = threading.Thread(target=self._monitoring_loop)
            self.monitor_thread.daemon = True
            self.monitor_thread.start()
            print(f"Started monitoring {self.server_url}")
        else:
            print("Monitoring is already running")

    def execute_instructions(self, json_data):
        if self._ongoing_movement.is_set():
            print("Waiting for movement(s) to complete before starting another")
        self._ongoing_movement.wait()
        print(f"Executing instruction {json_data}...")
        if json_data[0] == "a":
            self.motor_controller.move_controlled(
                json_data[1] if json_data[1] is not None else 50,
                event=self._ongoing_movement,
            )
        elif json_data[0] == "t":
            self.motor_controller.turn_controlled_deg(
                json_data[1] if json_data[1] is not None else 45,
                event=self._ongoing_movement,
            )
        elif json_data[0] == "c":
            pose = None
            self.tracking_server.capture_flag(json_data[1], pose)
            self.motor_controller.turn_controlled_deg(360, event=self._ongoing_movement)

    def stop_movement(self):
        self._ongoing_movement.set()

    def stop_monitoring(self):
        """Stop the monitoring thread."""
        self.is_running = False
        if self.monitor_thread:
            self.monitor_thread.join()
            print("Stopped monitoring")

    def _monitoring_loop(self):
        """Main monitoring loop that sends periodic checks."""
        while self.is_running:
            try:
                self._send_check()
                time.sleep(self.check_interval)
            except Exception as e:
                print(f"Unexpected error in monitoring loop: {str(e)}")
                time.sleep(self.check_interval)

    def _send_check(self):
        """Send a single check request to the server with retry logic."""
        retry_count = 0

        headers = {"id": str(self.id)}

        while retry_count < self.max_retries:
            try:
                start_time = time.time()
                response = requests.get(self.server_url, headers=headers, timeout=5)
                response_time = (
                    time.time() - start_time
                ) * 1000  # Convert to milliseconds

                if response.status_code == StrategyConfig.RESPONSE_SUCESS.value:
                    if self.consecutive_failures > 0:
                        print(
                            f"Connection restored after {self.consecutive_failures} failures!"
                        )
                    self.consecutive_failures = 0
                    # Only print every 10 successful checks to reduce output
                    if int(time.time()) % 10 == 0:
                        print(f"Server is UP - Response time: {response_time:.2f}ms")
                    return

                elif response.status_code == StrategyConfig.RESPONSE_MOVEMENT.value:
                    try:
                        json_data = response.json()
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

            except requests.exceptions.ConnectionError:
                self.consecutive_failures += 1
                retry_count += 1

                if retry_count < self.max_retries:
                    print(
                        f"ERROR - Connection failed - Retrying in {self.retry_delay} seconds... (Attempt {retry_count + 1}/{self.max_retries})"
                    )
                    time.sleep(self.retry_delay)
                else:
                    print(
                        f"ERROR - Connection failed after {self.max_retries} attempts - Server might be down"
                    )

            except requests.exceptions.Timeout:
                self.consecutive_failures += 1
                retry_count += 1

                if retry_count < self.max_retries:
                    print(
                        f"Request timed out - Retrying in {self.retry_delay} seconds... (Attempt {retry_count + 1}/{self.max_retries})"
                    )
                    time.sleep(self.retry_delay)
                else:
                    print(
                        f"ERROR - Request timed out after {self.max_retries} attempts"
                    )

            except Exception as e:
                print(f"ERROR - Unexpected error: {str(e)}")
                return


# Example usage
if __name__ == "__main__":
    # Create a monitor instance with custom retry settings
    monitor = GroupServerController(
        "http://example.com", check_interval=1.0, max_retries=3, retry_delay=5
    )

    try:
        # Start monitoring
        monitor.start_monitoring()

        # Keep the main thread running
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        # Handle graceful shutdown on Ctrl+C
        monitor.stop_monitoring()
        print("\nMonitoring stopped")
