import pathlib
import asyncio
import threading
import websockets
from aiohttp import web
import os
import json
from dataclasses import dataclass
from enum import Enum


class ClientType(Enum):
    ROBOT = "robot"
    INTERFACE = "interface"


class Client:
    def __init__(self, websocket, client_type):
        self.websocket = websocket
        self.client_type = client_type


class CombinedServer:
    def __init__(self, video_dir, vue_app_dir):
        self.http_port = 8080
        self.ws_port = 8765
        self.host = "0.0.0.0"
        self.video_dir = video_dir
        self.vue_app_dir = vue_app_dir
        self.http_server = None
        self.ws_server = None
        self.ws_clients = {}
        self.robot_id = None
        self.interfaces = set()
        self.stop_event = asyncio.Event()

    async def register_client(self, websocket, path: str):
        """Register a new WebSocket client connection"""
        try:
            message = await websocket.recv()
            data = json.loads(message)

            if "client_type" not in data:
                await websocket.close(1002, "Client type not specified")
                return

            client_type = ClientType(data["client_type"])
            client_id = str(id(websocket))

            self.ws_clients[client_id] = Client(websocket, client_type)
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
        if client_id not in self.ws_clients:
            return

        client = self.ws_clients[client_id]
        if client.client_type == ClientType.ROBOT:
            self.robot_id = None
        else:
            self.interfaces.remove(client_id)

        del self.ws_clients[client_id]
        print(f"{client.client_type.value} disconnected. ID: {client_id}")

    async def forward_to_robot(self, message: str):
        """Forward message to robot"""
        if self.robot_id and self.robot_id in self.ws_clients:
            try:
                await self.ws_clients[self.robot_id].websocket.send(message)
            except websockets.exceptions.ConnectionClosed:
                await self.unregister_client(self.robot_id)

    async def forward_to_interfaces(self, message: str):
        """Forward message to all web interfaces"""
        disconnected = set()
        for interface_id in self.interfaces:
            try:
                await self.ws_clients[interface_id].websocket.send(message)
            except websockets.exceptions.ConnectionClosed:
                disconnected.add(interface_id)

        for interface_id in disconnected:
            await self.unregister_client(interface_id)

    async def handle_client(self, client_id: str, websocket):
        """Handle messages from a client"""
        try:
            async for message in websocket:
                try:
                    data = json.loads(message)
                    client = self.ws_clients[client_id]

                    if client.client_type == ClientType.ROBOT:
                        await self.forward_to_interfaces(message)
                    else:
                        await self.forward_to_robot(message)

                except json.JSONDecodeError:
                    print(f"Invalid JSON received from {client_id}")

        except websockets.exceptions.ConnectionClosed:
            await self.unregister_client(client_id)

    async def start_ws_server(self):
        # Start the WebSocket server on a given port
        server = await websockets.serve(self.register_client, self.host, self.ws_port)
        print(f"WebSocket server started on ws://{self.host}:{self.ws_port}")
        await server.wait_closed()

    async def serve_video(self, request):
        # Serve the HLS video stream (m3u8 file)
        video_path = os.path.join(self.video_dir, "video.m3u8")
        if os.path.exists(video_path):
            return web.FileResponse(video_path)
        return web.Response(status=404, text="Video stream not found.")

    async def serve_vue_app(self, request):
        # Serve the Vue.js application index.html
        try:
            requested_path = request.match_info.get("path", "")
            full_path = os.path.normpath(os.path.join(self.vue_app_dir, requested_path))

            # Prevent directory traversal
            if not full_path.startswith(self.vue_app_dir):
                return web.Response(status=403, text="Forbidden")

            # If file exists, serve it directly
            if os.path.exists(full_path) and os.path.isfile(full_path):
                return web.FileResponse(full_path)

            # If no specific file, serve index.html
            index_path = os.path.join(self.vue_app_dir, "index.html")
            if os.path.exists(index_path):
                return web.FileResponse(index_path)

            return web.Response(status=404, text="Vue app not found.")

        except Exception as e:
            return web.Response(status=500, text=f"Server error: {str(e)}")
        # index_path = os.path.join(self.vue_app_dir, "index.html")
        # if os.path.exists(index_path):
        #     return web.FileResponse(index_path)
        # return web.Response(status=404, text="Vue app not found.")

    async def start_http_server(self):
        # Start the HTTP server
        app = web.Application()
        app.router.add_get("/video", self.serve_video)  # Serving m3u8 video stream

        # Add catch-all route for Vue app routing and static files
        app.router.add_get("/", self.serve_vue_app)
        app.router.add_get("/{path:.*}", self.serve_vue_app)

        runner = web.AppRunner(app)
        await runner.setup()
        site = web.TCPSite(runner, host=self.host, port=self.http_port)
        await site.start()
        print(f"HTTP server started on http://{self.host}:{self.http_port}")

    def start(self):
        """
        Start WebSocket and HTTP servers in separate asyncio event loops
        """
        self.main_loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.main_loop)

        async def run_servers():
            ws_server_task = asyncio.create_task(self.start_ws_server())
            http_server_task = asyncio.create_task(self.start_http_server())

            try:
                await asyncio.gather(ws_server_task, http_server_task)
            except asyncio.CancelledError:
                pass
            finally:
                self.stop_event.set()

        def run_async_loop():
            self.main_loop.run_until_complete(run_servers())

        # Start servers in a separate thread
        self.server_thread = threading.Thread(target=run_async_loop, daemon=True)
        self.server_thread.start()

    def stop(self):
        """
        Gracefully stop the servers
        """
        if hasattr(self, "main_loop") and not self.stop_event.is_set():
            # Cancel all running tasks
            for task in asyncio.all_tasks(self.main_loop):
                task.cancel()

            # Stop the event loop
            self.main_loop.call_soon_threadsafe(self.main_loop.stop)

            # Wait for the server thread to finish
            if hasattr(self, "server_thread"):
                self.server_thread.join(timeout=5)

            print("Servers stopped.")


def main(ws_port=8765, http_port=8000):
    import time

    server = CombinedServer(
        "./src/web/hls",
        "./src/web/web_interface",
    )
    try:
        server.start()
    except KeyboardInterrupt:
        print("\nServer shutdown requested")
    except Exception as e:
        print(f"Unexpected error: {e}")
    finally:
        while True:
            time.sleep(1)  # make sure the main thread does not stop
        print("Server stopped")


if __name__ == "__main__":
    main()
