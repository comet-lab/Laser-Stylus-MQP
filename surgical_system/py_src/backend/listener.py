import asyncio
import websockets
import os

class BackendConnection():
    def __init__(self, send_fn, recv_fn, mocking):
        self.send_fn = send_fn
        self.recv_fn = recv_fn
        self.mocking = mocking

    async def _send_loop(self, websocket):
        while True:
            msg = self.send_fn()
            if msg is not None:
                await websocket.send(msg)
            await asyncio.sleep(0.5)

    async def _recieve_loop(self, websocket):
        async for message in websocket:
            self.recv_fn(message)

    async def connect_to_websocket(self):
        host = "backend" if self.mocking else "localhost"
        port = "8080" if self.mocking else "443"
        ws_robot_name = os.getenv("ROBOT_WEBSOCKET_NAME", "robot")
        uri = f"ws://{host}:{port}/ws/{ws_robot_name}"
        async with websockets.connect(uri) as websocket:
            # Connection is now open
            print(f"Connected to WebSocket server at {uri}")

            await asyncio.gather(self._send_loop(websocket), self._recieve_loop(websocket))

    