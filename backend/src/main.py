from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
import json
import os

app = FastAPI()

#app.mount("/static", StaticFiles(directory="static"), name="static")

# Allow the frontend (served on localhost:3000) to call backend APIs during development
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# decided to make the backend save the current state
# the first three are for position and the second three are for orientation
current_data = {
    "x": 0,
    "y": 0,
    "z": 0,
    "rx": 0,
    "ry": 0,
    "rz":0,
    "laserX": 0,
    "laserY": 0,
    "beamWaist": 0,
    "speed": 0,
    "isLaserOn": False
}

@app.get("/media/{stream_name}")
def read_item(stream_name: str):
    return {"stream_url": "http://localhost:8889/mystream"}

@app.get("/health")
def health():
    return None

@app.post("/api/path")
async def submit_path(request: dict):
    pixels = request.get("pixels", [])
    
    if len(pixels) == 0:
        raise HTTPException(status_code=400, detail="No pixels provided")
    
    print(f"Received path with {len(pixels)} pixels")
    
    return {
        "status": "success",
        "pixel_count": len(pixels),
        "message": f"Path with {len(pixels)} pixels received and processed"
    }

# this code is from the FastAPI websockets documentation
class ConnectionManager:
    def __init__(self):
        self.frontend_connections: list[WebSocket] = []
        self.robot_connections: list[WebSocket] = []

    async def connect(self, websocket: WebSocket, group: str):
        await websocket.accept()
        if group == "frontend":
            self.frontend_connections.append(websocket)
        elif group == "robot":
            self.robot_connections.append(websocket)

    def disconnect(self, websocket: WebSocket):
        # was_connected tracks if the websocket is connected or not
        was_connected = False
        for group in (self.frontend_connections, self.robot_connections):
            if websocket in group:
                group.remove(websocket)
                was_connected = True

        if was_connected:
            print("connection disconnected. Turning laser off")
            current_data["isLaserOn"] = False

    async def broadcast_to_all(self, message: str):
        for connection in self.frontend_connections + self.robot_connections:
            try:
                await connection.send_text(message)
            except Exception:
                self.disconnect(connection)

manager = ConnectionManager()


ws_ui_name = os.getenv("UI_WEBSOCKET_NAME", "ui")
ws_robot_name = os.getenv("ROBOT_WEBSOCKET_NAME", "robot")

# this is the websocket to pass the 6 varaibles from frontend to backend
@app.websocket(f"/ws/{ws_ui_name}")
async def websocket_endpoint(websocket: WebSocket):
    await manager.connect(websocket, "frontend") 

    try:
        # sends the full connection when it connects
        await websocket.send_text(json.dumps(current_data))

        while True:
            data = await websocket.receive_text()
            try:
                message = json.loads(data)

                if not isinstance(message, dict):
                    await websocket.send_text("Error: Invalid JSON format")
                    continue
                # this updates only the keys that are sent from the frontend
                for key, value in message.items():
                    if key in current_data:
                        current_data[key] = value
                    else:
                        await websocket.send_text(f"Error: Unknown key '{key}'")
                # after updating, broadcast the full current data to all clients
                await manager.broadcast_to_all(json.dumps(current_data))

                print(f"Updated state: {current_data}")

            except json.JSONDecodeError:
                await websocket.send_text("Error: Invalid JSON format")

    except WebSocketDisconnect:
        manager.disconnect(websocket)
        print("Frontend disconnected.")
        await manager.broadcast_to_all(json.dumps(current_data))
        print("Frontend disconnected. Sending signals to turn laser off")

ws_robot_name = os.getenv("ROBOT_WEBSOCKET_NAME")

@app.websocket(f"/ws/{ws_robot_name}")
async def websocket_endpoint(websocket: WebSocket):
    await manager.connect(websocket, "robot") 

    try:
        # sends the full connection when it connects
        await websocket.send_text(json.dumps(current_data))

        while True:
            data = await websocket.receive_text()
            print(f"Robot sent data: {data}")
            await websocket.send_text("Got ur data")
    except WebSocketDisconnect:
        manager.disconnect(websocket)
        await manager.broadcast_to_all(json.dumps(current_data))
        print("Robot disconnected. Laser turned off")

# HTTP endpoint to get the current coordiantes
@app.get("/current-coordinates")
def get_current_coordinates():
    return current_data

# start with variables: position (like x and y coordinates) z, orientation: 
# have the socket accept websocket 
# make HTTP endpoints
# make an websockets 