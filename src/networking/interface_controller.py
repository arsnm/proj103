import json
import asyncio
import websockets


class InterfaceController:
    def __init__(self, uri):
        self.uri = uri
        self.websocket = None

    async def connect(self):
        """Establish connections to control and tracking servers."""
        self.websocket = await websockets.connect(self.uri)
        await self.websocket.send(
            json.dumps({"type": "connect", "data": {"client_type": "robot"}})
        )
        response = await self.websocket.recv()
        return json.loads(response)["data"]["id"]

    async def disconnect(self):
        """Clean up connections."""
        if self.websocket:
            try:
                await self.websocket.close()
            except:
                pass

    async def send_status(self, status):
        """Send robot status to control server."""
        if not self.websocket:
            return

        await self.websocket.send(
            json.dumps(
                {
                    "type": "status",
                    "data": {
                        "position": status.position,
                        "speed": status.speed,
                        "mode": status.mode,
                        "target": status.target,
                    },
                }
            )
        )

    async def handle_command(self, command: dict):
        """Handle incoming commands."""
        cmd_type = command.get("command")

        if cmd_type == "move":
            direction = command.get("direction")
            speed = int(command.get("speed", 0))

            if direction == "stop":
                motors.standby()
            elif (
                direction in ["forward", "backward", "left", "right"]
                and 1 <= speed <= 100
            ):
                self.motors.move_uncontrolled(direction, speed)

        elif cmd_type == "target":
            print(command)
            goal = (0, 0)
            if "case" in command:
                print("got_case")
                goal = self.case_to_coord(command["case"])
            elif "x" in command and "y" in command:
                goal = (float(command["x"]), float(command["y"]))
            await self.update_mode("target", goal)

        elif cmd_type == "mode":
            mode = command.get("mode_type")
            await self.update_mode(mode)
