from typing import Dict, Callable, Awaitable, Any
from models.message import MessageType


class MessageHandler:
    """Handles WebSocket message routing and processing"""

    def __init__(self):
        self.handlers: Dict[MessageType, Callable] = {}

    def register(
        self,
        msg_type: MessageType,
        handler: Callable[[Dict[str, Any]], Awaitable[None]],
    ):
        """Register a handler for a specific message type"""
        self.handlers[msg_type] = handler

    async def handle_message(self, msg_type: MessageType, data: Dict[str, Any]):
        """Route message to appropriate handler"""
        handler = self.handlers.get(msg_type)
        if handler:
            await handler(data)
        else:
            print(f"No handler for message type: {msg_type}")
