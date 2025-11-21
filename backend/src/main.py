from fastapi import FastAPI, HTTPException, WebSocket, UploadFile
from fastapi.middleware.cors import CORSMiddleware
import os
import base64
from robot import RobotSchema
from manager import ConnectionManager

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

manager = ConnectionManager()

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
    manager.desired_state.path = pixels
    await manager.broadcast_to_group(group=manager.robot_connections, state=manager.desired_state)
    
    return {
        "status": "success",
        "pixel_count": len(pixels),
        "message": f"Path with {len(pixels)} pixels dispatched to robot"
    }

@app.post("/api/raster_mask")   
async def submit_raster_mask(file: UploadFile):
    extension = file.filename.split('.')[-1]
    if(file.filename.split('.')[-1] != "png"):
        raise HTTPException(status_code = 400, detail=f"Incompatible file extension: {extension}")
    
    bytes = file.file.read()
    encoded_b64 = base64.b64encode(bytes) # png is base 64 encoded
    encoded_utf8 = encoded_b64.decode('utf-8') # re-encode to utf-8 to send over websocket
    
    manager.desired_state.raster_mask = encoded_utf8
    print("Sending raster")
    await manager.broadcast_to_group(group=manager.robot_connections, state=manager.desired_state)
    return file.filename

manager = ConnectionManager()

# this is the websocket to pass the 6 varaibles from frontend to backend
ws_ui_name = os.getenv("UI_WEBSOCKET_NAME", "ui")
@app.websocket(f"/ws/{ws_ui_name}")
async def websocket_endpoint(websocket: WebSocket):
    await manager.manage(
        websocket=websocket,
        state=manager.desired_state,
        connection_group=manager.frontend_connections,
        forwarding_group=manager.robot_connections
    )

ws_robot_name = os.getenv("ROBOT_WEBSOCKET_NAME", "robot")
@app.websocket(f"/ws/{ws_robot_name}")
async def websocket_endpoint(websocket: WebSocket):
    await manager.manage(
        websocket=websocket,
        state=manager.current_state,
        connection_group=manager.robot_connections,
        forwarding_group=manager.frontend_connections
    )

# HTTP endpoint to get the current coordiantes
@app.get("/current-coordinates")
def get_current_coordinates():
    return manager.current_state.to_str()

# start with variables: position (like x and y coordinates) z, orientation: 
# have the socket accept websocket 
# make HTTP endpoints
# make an websockets 