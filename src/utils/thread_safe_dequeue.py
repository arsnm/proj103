from threading import Lock, Condition, Event
from typing import Optional, List
from collections import deque


class ThreadSafeDeque:
    """
    A thread-safe implementation of a double-ended queue (deque) with blocking operations.
    All operations are protected by a lock to ensure thread safety.
    """

    def __init__(self, maxlen: Optional[int] = None):
        """
        Initialize the thread-safe deque.

        Args:
            maxlen: Optional maximum length of the deque
        """
        self._deque = deque(maxlen=maxlen)
        self._lock = Lock()
        self._not_empty = Condition(self._lock)
        self._unfinished_tasks = 0
        self._tasks_lock = Lock()
        self._clear_event = Event()

    def append(self, item: object) -> None:
        """Add an item to the right end of the deque."""
        with self._lock:
            self._deque.append(item)
            self._unfinished_tasks += 1
            self._not_empty.notify()

    def appendleft(self, item: object) -> None:
        """Add an item to the left end of the deque."""
        with self._lock:
            self._deque.appendleft(item)
            self._unfinished_tasks += 1
            self._not_empty.notify()

    def pop(self, timeout: Optional[float] = None) -> object:
        """
        Remove and return an item from the right end.
        Blocks until an item is available if the deque is empty.

        Args:
            timeout: Optional timeout in seconds. If None, blocks indefinitely

        Returns:
            The rightmost item from the deque

        Raises:
            TimeoutError: If timeout is reached before an item becomes available
        """
        with self._lock:
            while len(self._deque) == 0:
                if not self._not_empty.wait(timeout=timeout):
                    raise TimeoutError("No item available within the timeout period")
            return self._deque.pop()

    def popleft(self, timeout: Optional[float] = None) -> object:
        """
        Remove and return an item from the left end.
        Blocks until an item is available if the deque is empty.

        Args:
            timeout: Optional timeout in seconds. If None, blocks indefinitely

        Returns:
            The leftmost item from the deque

        Raises:
            TimeoutError: If timeout is reached before an item becomes available
        """
        with self._lock:
            while len(self._deque) == 0:
                if not self._not_empty.wait(timeout=timeout):
                    raise TimeoutError("No item available within the timeout period")
            return self._deque.popleft()

    def task_done(self) -> None:
        """
        Indicate that a previously enqueued task is complete.
        Used in conjunction with join().

        Raises:
            ValueError: If called more times than there are items
        """
        with self._tasks_lock:
            unfinished = self._unfinished_tasks - 1
            if unfinished < 0:
                raise ValueError("task_done() called too many times")
            self._unfinished_tasks = unfinished
            if unfinished == 0:
                with self._lock:
                    self._not_empty.notify_all()

    def join(self, timeout: Optional[float] = None) -> None:
        """
        Block until all items in the deque have been processed.
        An item is processed when task_done() is called for it.

        Args:
            timeout: Optional timeout in seconds. If None, blocks indefinitely

        Raises:
            TimeoutError: If timeout is reached before all tasks are done
        """
        with self._lock:
            while self._unfinished_tasks > 0:
                if not self._not_empty.wait(timeout=timeout):
                    raise TimeoutError("Join timeout: tasks still pending")

    def clear(self) -> None:
        """Remove all elements from the deque."""
        with self._lock:
            self._clear_event.set()
            self._deque.clear()
            self._unfinished_tasks = 0

    def acknowledge_clear(self) -> None:
        """Acknowledge that the dequeue as been cleared."""
        with self._lock:
            self._clear_event.clear()

    def extend(self, iterable) -> None:
        """Extend the right side of the deque with elements from the iterable."""
        with self._lock:
            count = 0
            for item in iterable:
                self._deque.append(item)
                count += 1
            self._unfinished_tasks += count
            if count > 0:
                self._not_empty.notify()

    def extendleft(self, iterable) -> None:
        """Extend the left side of the deque with elements from the iterable."""
        with self._lock:
            count = 0
            for item in iterable:
                self._deque.appendleft(item)
                count += 1
            self._unfinished_tasks += count
            if count > 0:
                self._not_empty.notify()

    def __len__(self) -> int:
        """Return the number of elements in the deque."""
        with self._lock:
            return len(self._deque)

    def __bool__(self) -> bool:
        """Return True if the deque has elements, False otherwise."""
        with self._lock:
            return bool(self._deque)

    def peek(self) -> Optional[object]:
        """
        Return the rightmost element without removing it.
        Returns None if deque is empty.
        """
        with self._lock:
            return self._deque[-1] if self._deque else None

    def peekleft(self) -> Optional[object]:
        """
        Return the leftmost element without removing it.
        Returns None if deque is empty.
        """
        with self._lock:
            return self._deque[0] if self._deque else None

    def to_list(self) -> List[object]:
        """Return a list of all elements in the deque."""
        with self._lock:
            return list(self._deque)
