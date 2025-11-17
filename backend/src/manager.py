import json
from robot import RobotSchema
from fastapi import WebSocket, WebSocketDisconnect

class ConnectionManager:
    class ConnectionGroup(list):
        def __init__(self, name: str):
            super().__init__()
            self.name = name

    def __init__(self):
        self.frontend_connections = ConnectionManager.ConnectionGroup("Frontend")
        self.robot_connections = ConnectionManager.ConnectionGroup("Robot")
        self.current_state = RobotSchema()
        self.desired_state = RobotSchema()

    async def connect(self, websocket: WebSocket, group):
        await websocket.accept()
        group.append(websocket)

    def disconnect(self, websocket: WebSocket):
        print("Websocket disconnected. Turning laser off")
        self.desired_state.isLaserOn = False
        for group in (self.frontend_connections, self.robot_connections):
            if websocket in group:
                group.remove(websocket)     

    async def broadcast_to_all(self, message: str):
        for connection in self.frontend_connections + self.robot_connections:
            try:
                await connection.send_text(message)
            except Exception:
                self.disconnect(connection)

    async def broadcast_to_group(self, group: ConnectionGroup, message):
        for connection in group:
            try:
                await connection.send_text(message)
            except Exception:
                self.disconnect(connection)

    async def manage(self, websocket, state: RobotSchema, connection_group: ConnectionGroup, forwarding_group: ConnectionGroup):
        """ Handle the lifecycle of a websocket - consume message and forward to subscribers.

        Args:
            websocket: websocket to handle.
            state: Target robot state to update and pass along.
            connection_group: Connection Manager group to join.
            forwarding_group: Connection Manager group to send messages to.
        """
        await self.connect(websocket, connection_group) 

        try:
            while True:
                data = await websocket.receive_text()
                try:
                    message = json.loads(data)
                    
                    if not isinstance(message, dict):
                        print("Error: Invalid JSON format")
                        print(message)
                        continue    
                    # this updates only the keys that are sent from the frontend
                    state.update(message)
                    await self.broadcast_to_group(forwarding_group, state.to_str())

                except json.JSONDecodeError:
                    print("Error: Invalid JSON format")
                    print(message)

        except WebSocketDisconnect:
            self.disconnect(websocket)
            print("Websocket disconnected. Sending signals to turn laser off")
            await self.broadcast_to_group(self.robot_connections, self.desired_state.to_str())
