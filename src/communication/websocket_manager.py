import asyncio
import websockets
import cv2
import base64
from typing import Dict, Callable, Optional
from models.message import Message, MessageType
from models.position import Position
from .message_handler import MessageHandler
from websockets.asyncio.client import ClientConnection
import json


class WebSocketManager:
    """Manages WebSocket communication with server"""

    def __init__(self, uri: str):
        self.uri = uri
        self.websocket: Optional[ClientConnection]
        self.running = True
        self.message_handlers: Dict[MessageType, Callable] = {}
        self.message_queue = asyncio.Queue()
        self.connected = False
        self.message_handler = MessageHandler()

    def initialize_connection(self):
        """Initialize WebSocket connection"""
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(self.connect())

    async def connect(self):
        """Establish WebSocket connection with reconnection logic"""
        while self.running:
            try:
                self.websocket = await websockets.connect(self.uri)
                await self.websocket.send(json.dumps({"client_type": "robot"}))
                self.connected = True
                print(f"Connected to server at {self.uri}")
                await self.handle_connection()
            except websockets.exceptions.ConnectionClosed:
                print("Connection lost. Reconnecting...")
                self.connected = False
                await asyncio.sleep(1)
            except Exception as e:
                print(f"Connection error: {e}")
                self.connected = False
                await asyncio.sleep(1)

    async def handle_connection(self):
        """Handle active WebSocket connection"""
        receive_task = asyncio.create_task(self.receive_messages())
        send_task = asyncio.create_task(self.send_messages())

        await asyncio.gather(receive_task, send_task)

    async def receive_messages(self):
        """Process incoming messages"""
        while self.running and self.websocket:
            try:
                message = await self.websocket.recv()
                await self.process_message(str(message))
            except websockets.exceptions.ConnectionClosed:
                break
            except Exception as e:
                print(f"Error receiving message: {e}")

    async def send_messages(self):
        """Send queued messages"""
        while self.running and self.websocket:
            try:
                message = await self.message_queue.get()
                await self.websocket.send(message)
            except Exception as e:
                print(f"Error sending message: {e}")
                await asyncio.sleep(0.1)

    def send_robot_status(self, status: Dict):
        """Send robot status update"""
        message = Message(type=MessageType.ROBOT_STATUS, data=status)
        self.message_queue.put_nowait(message.to_json())

    def send_position_update(self, position: Position):
        """Send position update"""
        message = Message(
            type=MessageType.POSITION_UPDATE,
            data={
                "x": position.x,
                "y": position.y,
                "theta": position.theta,
                "camera_angle": position.camera_angle,
                "confidence": position.confidence,
            },
        )
        self.message_queue.put_nowait(message.to_json())

    def send_video_frame(self, frame):
        """Send camera frame"""
        _, buffer = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        frame_data = base64.b64encode(buffer).decode("utf-8")

        message = Message(type=MessageType.VIDEO_FRAME, data={"frame": frame_data})
        self.message_queue.put_nowait(message.to_json())
        print("[WEBSOCKET] - Added video frame to message queue.")

    def register_handler(self, msg_type: MessageType, handler: Callable):
        self.message_handler.register(msg_type, handler)

    async def process_message(self, message_str: str):
        try:
            message = Message.from_json(message_str)
            await self.message_handler.handle_message(message.type, message.data)
        except Exception as e:
            print(f"Error processing message: {e}")

    def stop(self):
        """Stop WebSocket communication"""
        self.running = False
        if self.websocket:
            asyncio.create_task(self.websocket.close())
