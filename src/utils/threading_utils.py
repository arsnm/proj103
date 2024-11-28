import threading
import queue
import signal
from typing import List, Callable
from dataclasses import dataclass
import time


@dataclass
class ThreadInfo:
    """Information about a controlled thread"""

    name: str
    thread: threading.Thread
    frequency: float  # Hz
    running: bool = True
    last_execution: float = 0.0


class ThreadController:
    """Manages multiple threads with proper initialization and cleanup"""

    def __init__(self):
        self.threads: List[ThreadInfo] = []
        self.stop_event = threading.Event()
        self._setup_signal_handling()

    def _setup_signal_handling(self):
        """Setup signal handlers for graceful shutdown"""
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def _signal_handler(self, signum, frame):
        """Handle shutdown signals"""
        print("\nShutting down threads...")
        self.stop_all()

    def add_thread(self, name: str, target: Callable, frequency: float) -> ThreadInfo:
        """
        Add a new thread to be managed
        Args:
            name: Thread identifier
            target: Function to run in thread
            frequency: Desired execution frequency in Hz
        """

        def wrapper():
            period = 1.0 / frequency
            thread_info = next(t for t in self.threads if t.name == name)

            while not self.stop_event.is_set() and thread_info.running:
                start_time = time.time()

                try:
                    target()
                except Exception as e:
                    print(f"Error in thread {name}: {e}")
                    # Continue running unless it's a critical error
                    if isinstance(e, (KeyboardInterrupt, SystemExit)):
                        break

                # Maintain desired frequency
                thread_info.last_execution = time.time()
                elapsed = thread_info.last_execution - start_time
                sleep_time = max(0, period - elapsed)
                if sleep_time > 0:
                    time.sleep(sleep_time)
                elif elapsed > period * 1.5:  # Significant delay
                    print(
                        f"Warning: {name} thread running slower than desired frequency"
                    )

        thread = threading.Thread(target=wrapper, name=name, daemon=True)
        thread_info = ThreadInfo(name=name, thread=thread, frequency=frequency)
        self.threads.append(thread_info)
        return thread_info

    def start_all(self):
        """Start all registered threads"""
        self.stop_event.clear()
        for thread_info in self.threads:
            if not thread_info.thread.is_alive():
                thread_info.running = True
                thread_info.thread.start()

    def stop_all(self):
        """Stop all threads gracefully"""
        self.stop_event.set()
        for thread_info in self.threads:
            thread_info.running = False
            if thread_info.thread.is_alive():
                thread_info.thread.join(timeout=1.0)

    def is_running(self) -> bool:
        """Check if any thread is still running"""
        return any(t.running and t.thread.is_alive() for t in self.threads)

    def get_thread_status(self) -> List[dict]:
        """Get status of all threads"""
        current_time = time.time()
        return [
            {
                "name": t.name,
                "alive": t.thread.is_alive(),
                "running": t.running,
                "frequency": t.frequency,
                "last_execution": (
                    current_time - t.last_execution if t.last_execution > 0 else None
                ),
            }
            for t in self.threads
        ]


class SafeQueue(queue.Queue):
    """Thread-safe queue with timeout and error handling"""

    def get_nowait_safe(self, default=None):
        """
        Get item from queue without blocking
        Returns default if queue is empty
        """
        try:
            return self.get_nowait()
        except queue.Empty:
            return default

    def put_nowait_safe(self, item) -> bool:
        """
        Put item in queue without blocking
        Returns True if successful, False if queue is full
        """
        try:
            self.put_nowait(item)
            return True
        except queue.Full:
            return False


# Example usage:
if __name__ == "__main__":

    def example_thread():
        print(f"Thread running at {time.time()}")

    controller = ThreadController()

    # Add threads with different frequencies
    controller.add_thread("fast_thread", example_thread, frequency=10.0)  # 10 Hz
    controller.add_thread("slow_thread", example_thread, frequency=1.0)  # 1 Hz

    try:
        controller.start_all()
        while controller.is_running():
            time.sleep(1)
            status = controller.get_thread_status()
            print("Thread status:", status)

    except KeyboardInterrupt:
        print("Stopping threads...")
    finally:
        controller.stop_all()
