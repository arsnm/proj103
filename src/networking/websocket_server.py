import asyncio
import websockets
import os
from datetime import datetime
import json
from dataclasses import dataclass
from aiohttp import web


@dataclass
class RobotState:
    id: int
    position = (0.0, 0.0, 0.0)  # x, y, angle
    speed = (0.0, 0.0)
    mode = "manual"

    def to_dict(self):
        return {
            "id": self.id,
            "position": self.position,
            "speed": self.speed,
            "mode": self.mode,
        }


class Server:
    def __init__(self, host="0.0.0.0", ws_port=8765, http_port=8080, web_dir="static"):
        self.host = host
        self.ws_port = ws_port
        self.http_port = http_port
        self.web_dir = os.path.join(os.path.dirname(__file__), web_dir)
        self.robots = {}
        self.robot_connections = {}
        self.client_connections = {}
        self.available_ids = {"robot": set(), "client": set()}

    async def broadcast_to_clients(self, message: dict):
        if self.client_connections:
            message_str = json.dumps(message)
            await asyncio.gather(
                *[
                    client.send(message_str)
                    for client in self.client_connections.values()
                ]
            )

    async def handle_robot_message(self, robot_id: int, message: dict):
        try:
            msg_type = message["type"]
            data = message.get("data", {})
            if robot_id not in self.robots:
                print(f"ERROR - Robot {robot_id} sent a command while disconnected.")
            else:
                robot = self.robots[robot_id]

                if msg_type == "status":
                    for key, value in data.items():
                        setattr(robot, key, value)
                    await self.broadcast_to_clients(
                        {"type": "robot_status", "data": robot.to_dict()}
                    )

                elif msg_type == "camera_frame":
                    robot.frame_data = data
                    await self.broadcast_to_clients(
                        {
                            "type": "camera_frame",
                            "data": {"id": robot_id, "frame": data},
                        }
                    )

                elif msg_type == "disconnect":
                    await self.handle_disconnect("robot", robot_id)
        except Exception as e:
            print(f"ERROR - Robot message error: {str(e)}", robot_id)

    async def handle_client_message(self, client_id: int, message: dict):
        try:
            msg_type = message["type"]
            data = message.get("data", {})

            if msg_type == "command":
                robot_id = int(data["robot_id"])
                if robot_id in self.robot_connections:
                    await self.robot_connections[robot_id].send(json.dumps(message))
                    print(f"Client {client_id} sent command to robot {robot_id}")

            elif msg_type == "get_status":
                robot_id = int(data["robot_id"])
                if robot_id in self.robots:
                    robot = self.robots[robot_id]
                    await self.client_connections[client_id].send(
                        json.dumps({"type": "robot_status", "data": robot.to_dict()})
                    )

            elif msg_type == "disconnect":
                if data["client_type"] == "client":
                    await self.handle_disconnect("client", client_id)
                elif data["client_type"] == "robot":
                    robot_id = int(data["robot_id"])
                    if robot_id in self.robot_connections:
                        # Send disconnect message to robot before closing
                        try:
                            await self.robot_connections[robot_id].send(
                                json.dumps({"type": "force_disconnect"})
                            )
                        except:
                            pass
                        await self.handle_disconnect("robot", robot_id)
                        print(
                            f"Robot {robot_id} was disconnected by client {client_id}"
                        )

        except Exception as e:
            print(f"ERROR - Client message error: {str(e)}")

    async def handle_disconnect(self, conn_type: str, conn_id: int):
        if conn_type == "robot":
            if conn_id in self.robots:
                del self.robots[conn_id]
                del self.robot_connections[conn_id]
                self.available_ids["robot"].add(conn_id)
                await self.broadcast_to_clients(
                    {"type": "robot_list", "data": {"robots": list(self.robots.keys())}}
                )
        else:
            if conn_id in self.client_connections:
                del self.client_connections[conn_id]
                self.available_ids["client"].add(conn_id)

    async def handle_websocket(self, websocket):
        """Handle WebSocket connections."""
        try:
            message = await websocket.recv()
            data = json.loads(message)

            if data["type"] != "connect":
                raise ValueError("First message must be a connect message")

            conn_type = data["data"]["client_type"]
            conn_id = min(
                self.available_ids[conn_type]
                or {len(getattr(self, f"{conn_type}_connections")) + 1}
            )

            if conn_type == "robot":
                self.robots[conn_id] = RobotState(id=conn_id)
                self.robot_connections[conn_id] = websocket
                await self.broadcast_to_clients(
                    {"type": "robot_list", "data": {"robots": list(self.robots.keys())}}
                )
            else:
                self.client_connections[conn_id] = websocket
                await websocket.send(
                    json.dumps(
                        {
                            "type": "robot_list",
                            "data": {"robots": list(self.robots.keys())},
                        }
                    )
                )

            await websocket.send(
                json.dumps({"type": "connected", "data": {"id": conn_id}})
            )
            print(f"New {conn_type} connection")

            async for message in websocket:
                if message == "ping":
                    print("received a ping")
                else:
                    data = json.loads(message)
                    if conn_type == "robot":
                        await self.handle_robot_message(conn_id, data)
                    else:
                        await self.handle_client_message(conn_id, data)

        except websockets.exceptions.ConnectionClosed:
            if "conn_type" in locals():
                if conn_type == "robot":
                    await self.handle_disconnect("robot", conn_id)
                else:
                    await self.handle_disconnect("client", conn_id)
        except Exception as e:
            print(f"ERROR - WebSocket error: {str(e)}")

    async def init_app(self):
        """Initialize the HTTP application."""
        app = web.Application()
        app.router.add_static(
            "/static", self.web_dir
        )  # Serve static files, including HLS stream

        # Serve the index.html file
        async def index(request):
            index_path = os.path.join(self.web_dir, "index.html")
            return web.FileResponse(index_path)

        app.router.add_get("/", index)
        return app

    async def start(self):
        """Start both HTTP and WebSocket servers."""
        # Start HTTP server
        app = await self.init_app()
        runner = web.AppRunner(app)
        await runner.setup()
        site = web.TCPSite(runner, self.host, self.http_port)
        await site.start()
        print(f"HTTP server started on http://{self.host}:{self.http_port}")

        # Start WebSocket server
        ws_server = await websockets.serve(
            self.handle_websocket, self.host, self.ws_port
        )
        print(f"WebSocket server started on ws://{self.host}:{self.ws_port}")

        # Keep the servers running
        await asyncio.Future()


def main():
    server = Server()
    asyncio.run(server.start())


if __name__ == "__main__":
    main()
