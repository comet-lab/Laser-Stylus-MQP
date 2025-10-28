from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles
import json

app = FastAPI()

app.mount("/static", StaticFiles(directory="static"), name="static")

@app.get("/media/{stream_name}")
def read_item(stream_name: str):
    return {"stream_url": "http://localhost:8889/mystream"}

# this code is from the FastAPI websockets documentation
class ConnectionManager:
    def __init__(self):
        self.active_connections: list[WebSocket] = []

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)

    def disconnect(self, websocket: WebSocket):
        self.active_connections.remove(websocket)

    async def send_personal_message(self, message: str, websocket: WebSocket):
        await websocket.send_text(message)

    async def broadcast(self, message: str):
        for connection in self.active_connections:
            await connection.send_text(message)

manager = ConnectionManager()

# the first three are for position and the second three are for orientation
example_data = {
    "x": 0,
    "y": 0,
    "z": 0,
    "rx": 0,
    "ry": 0,
    "rz":0
}

# this is the websocket to pass the 6 varaibles from frontend to backend
@app.websocket("/ws/coordinates")
async def websocket_endpoint(websocket: WebSocket):
    await manager.connect(websocket) 

    try:
        while True:
            data = await websocket.receive_text()
            try:
                coords = json.loads(data)
                required_keys = ["x", "y", "z", "rx", "ry", "rz"]

                # makes sure that the corerct keys are present
                if not all(key in coords for key in required_keys):
                    await websocket.send_text("Error: Missing keys")
                    continue
                # makes sure values are numbers
                if not all(isinstance(coords[key], (int, float)) for key in required_keys):
                    await websocket.send_text("Error: Values need to be numbers")
                    continue

                # updates the example data
                for key in required_keys:
                    example_data[key] = float(coords[key])

                # broadcasts the data as a json string
                await manager.broadcast(json.dumps(example_data))
                # print for debugging testing
                print(f"Updated coordinates: {example_data}")
            except json.JSONDecodeError:
                await websocket.send_text("Error: Invalid JSON format.")
    except WebSocketDisconnect:
        manager.disconnect(websocket)
        print("Client disconnected.")

# HTTP endpoint to get the current coordiantes
@app.get("/current-coordinates")
def get_current_coordinates():
    return example_data

# start with variables: position (like x and y coordinates) z, orientation: 
# have the socket accept websocket 
# make HTTP endpoints
# make an websockets 
