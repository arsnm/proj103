import asyncio
import websockets
import json
from typing import Dict, Set
from dataclasses import dataclass
from enum import Enum
import pathlib
from aiohttp import web
import os


class ClientType(Enum):
    ROBOT = "robot"
    INTERFACE = "interface"


@dataclass
class Client:
    websocket: websockets.WebSocketServerProtocol
    client_type: ClientType


class CombinedServer:
    def __init__(
        self, host: str = "localhost", ws_port: int = 8765, http_port: int = 8000
    ):
        self.host = host
        self.ws_port = ws_port
        self.http_port = http_port
        self.clients: Dict[str, Client] = {}
        self.robot_id = None
        self.interfaces: Set[str] = set()

        # Web interface directory
        self.web_dir = pathlib.Path(__file__).parent / "web_interface"

        # Create aiohttp app
        self.app = web.Application()
        self.setup_routes()

    def setup_routes(self):
        """Setup HTTP routes"""
        self.app.router.add_get("/", self.serve_index)
        # Serve static files
        self.app.router.add_static(
            "/static/", path=self.web_dir / "static", name="static"
        )

    async def serve_index(self, request):
        """Serve index.html"""
        return web.FileResponse(self.web_dir / "index.html")

    async def register_client(self, websocket: websockets.WebSocketServerProtocol):
        """Register a new WebSocket client connection"""
        try:
            message = await websocket.recv()
            data = json.loads(message)

            if "client_type" not in data:
                await websocket.close(1002, "Client type not specified")
                return

            client_type = ClientType(data["client_type"])
            client_id = str(id(websocket))

            self.clients[client_id] = Client(websocket, client_type)
            print(f"New {client_type.value} connected. ID: {client_id}")

            if client_type == ClientType.ROBOT:
                if self.robot_id:
                    await websocket.close(1002, "Robot already connected")
                    return
                self.robot_id = client_id
            else:
                self.interfaces.add(client_id)

            await self.handle_client(client_id, websocket)

        except Exception as e:
            print(f"Error during client registration: {e}")

    async def unregister_client(self, client_id: str):
        """Unregister a client connection"""
        if client_id not in self.clients:
            return

        client = self.clients[client_id]
        if client.client_type == ClientType.ROBOT:
            self.robot_id = None
        else:
            self.interfaces.remove(client_id)

        del self.clients[client_id]
        print(f"{client.client_type.value} disconnected. ID: {client_id}")

    async def forward_to_robot(self, message: str):
        """Forward message to robot"""
        if self.robot_id and self.robot_id in self.clients:
            try:
                await self.clients[self.robot_id].websocket.send(message)
            except websockets.exceptions.ConnectionClosed:
                await self.unregister_client(self.robot_id)

    async def forward_to_interfaces(self, message: str):
        """Forward message to all web interfaces"""
        disconnected = set()
        for interface_id in self.interfaces:
            try:
                await self.clients[interface_id].websocket.send(message)
            except websockets.exceptions.ConnectionClosed:
                disconnected.add(interface_id)

        for interface_id in disconnected:
            await self.unregister_client(interface_id)

    async def handle_client(
        self, client_id: str, websocket: websockets.WebSocketServerProtocol
    ):
        """Handle messages from a client"""
        try:
            async for message in websocket:
                try:
                    data = json.loads(message)
                    client = self.clients[client_id]

                    if client.client_type == ClientType.ROBOT:
                        await self.forward_to_interfaces(message)
                    else:
                        await self.forward_to_robot(message)

                except json.JSONDecodeError:
                    print(f"Invalid JSON received from {client_id}")

        except websockets.exceptions.ConnectionClosed:
            await self.unregister_client(client_id)

    async def start(self):
        """Start both WebSocket and HTTP servers"""
        # Start WebSocket server
        websocket_server = await websockets.serve(
            self.register_client, self.host, self.ws_port
        )
        print(f"WebSocket server running on ws://{self.host}:{self.ws_port}")

        # Start HTTP server
        runner = web.AppRunner(self.app)
        await runner.setup()
        site = web.TCPSite(runner, self.host, self.http_port)
        await site.start()
        print(f"HTTP server running on http://{self.host}:{self.http_port}")

        # Keep servers running
        try:
            await asyncio.Future()  # run forever
        finally:
            websocket_server.close()
            await websocket_server.wait_closed()
            await runner.cleanup()


def main():
    server = CombinedServer()

    try:
        asyncio.run(server.start())
    except KeyboardInterrupt:
        print("\nServer shutdown requested")
    except Exception as e:
        print(f"Unexpected error: {e}")
    finally:
        print("Server stopped")


if __name__ == "__main__":
    main()
