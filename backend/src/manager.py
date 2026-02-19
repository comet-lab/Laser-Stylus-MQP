#backend/src/manager.py

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
        self.desired_state.isLaserOn = False
        self.desired_state.isRobotOn = False
        self.desired_state.isTransformedViewOn = True # Default to Transformed
        self.desired_state.isThermalViewOn = False
        self.desired_state.speed = 10.0

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

    async def broadcast_to_group(self, group: ConnectionGroup, state: RobotSchema):
        for connection in group:
            try:
                await connection.send_text(state.flush())
            except Exception:
                self.disconnect(connection)

    async def manage(self, websocket, state: RobotSchema, connection_group: ConnectionGroup, forwarding_group: ConnectionGroup):
        """Handle the lifecycle of a websocket — consume message and forward to subscribers."""
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

                    # Update the robot's state
                    if forwarding_group == self.robot_connections:
                        # print(message)
                        pass
                    state.update(message)
                    await self.broadcast_to_group(forwarding_group, state)
                    if(forwarding_group == self.frontend_connections):
                        if('path_preview' in message.keys() and message['path_preview'] is not None):
                            message['path_preview']['time'] = message['path_preview']['time'][0]
                            print("TIME:",message['path_preview']['time'])
                    
                    # RESET EVENT: 
                    # If we just broadcasted a pathEvent (start/end), reset it to None 
                    # so it doesn't persist in future coordinate updates.
                    if state.pathEvent is not None:
                        state.pathEvent = None

                except json.JSONDecodeError:
                    print("Error: Invalid JSON format")
                    print(message)

        except WebSocketDisconnect:
            self.disconnect(websocket)
            print("Websocket disconnected. Sending signals to turn laser off")
            await self.broadcast_to_group(self.robot_connections, self.desired_state)
