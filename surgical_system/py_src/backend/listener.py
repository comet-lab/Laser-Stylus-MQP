import asyncio
import websockets
import os
from websockets.exceptions import ConnectionClosedError

class BackendConnection():
    def __init__(self, send_fn, recv_fn, mocking, refresh_ms = 50):
        self.send_fn = send_fn
        self.recv_fn = recv_fn
        self.mocking = mocking
        self.refresh_ms = refresh_ms

    async def _send_loop(self, websocket):
        while True:
            msg = self.send_fn()
            if msg is not None:
                await websocket.send(msg)
            await asyncio.sleep(self.refresh_ms / 1000.0)

    async def _recieve_loop(self, websocket):
        async for message in websocket:
            self.recv_fn(message)

    async def connect_to_websocket(self):
        host = "backend" if self.mocking else "localhost"
        port = "8080" if self.mocking else "443"
        ws_robot_name = os.getenv("ROBOT_WEBSOCKET_NAME", "robot")
        print(host, port, ws_robot_name)
        uri = f"ws://{host}:{port}/ws/{ws_robot_name}"
        while True:
            try:
                async with websockets.connect(uri) as websocket:
                # Connection is now open
                    print(f"Connect to WebSocket server at {uri}")
                    await asyncio.gather(self._send_loop(websocket), self._recieve_loop(websocket))
            except ConnectionClosedError:
                # TODO shutdown sequence if this occurs
                print("Connection closed. Reconnecting...")
            except ConnectionRefusedError:
                print("Connection refused. Retrying...")
            await asyncio.sleep(5)



    