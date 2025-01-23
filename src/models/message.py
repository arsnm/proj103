from dataclasses import dataclass
from enum import Enum
from typing import Any, Dict, Optional
import json
import time


class MessageType(Enum):
    # Robot -> Server -> Interface messages
    ROBOT_STATUS = "robot_status"

    # Interface -> Robot messages
    MODE_CHANGE = "mode_change"
    MANUAL_COMMAND = "manual_command"

    # System messages
    ERROR = "error"
    CONNECTION_STATUS = "connection_status"


@dataclass
class Message:
    """Base message structure for WebSocket communication"""

    type: MessageType
    data: Dict[str, Any]
    timestamp: Optional[float] = None

    def __post_init__(self):
        if self.timestamp is None:
            self.timestamp = time.time()

    def to_json(self) -> str:
        """Convert message to JSON string"""
        return json.dumps(
            {"type": self.type.value, "data": self.data, "timestamp": self.timestamp}
        )

    @classmethod
    def from_json(cls, json_str: str) -> "Message":
        """Create message from JSON string"""
        try:
            data = json.loads(json_str)
            return cls(
                type=MessageType(data["type"]),
                data=data["data"],
                timestamp=data["timestamp"],
            )
        except Exception as e:
            raise ValueError(f"Invalid message format: {e}")
