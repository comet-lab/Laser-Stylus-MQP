from fastapi import FastAPI, HTTPException, WebSocket, UploadFile, Request, Form, File
from fastapi.middleware.cors import CORSMiddleware
import os
import base64
from robot import RobotSchema
from manager import ConnectionManager
import json

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

@app.post("/api/execute")
async def execute_bundled_command(
    speed: float = Form(...),
    raster_type: str = Form(...),
    pixels: str = Form(...),
    file: UploadFile = File(...)
):
    # Parse the pixel path
    try:
        pixel_list = json.loads(pixels)
    except json.JSONDecodeError:
        raise HTTPException(status_code=400, detail="Invalid pixel data format")

    # Process and save the raster mask
    file_content = await file.read()
    save_directory = "saved_masks"
    os.makedirs(save_directory, exist_ok=True)
    file_location = f"{save_directory}/{file.filename}"
    
    with open(file_location, "wb") as f:
        f.write(file_content)

    # Encode for websocket transmission
    encoded_utf8 = base64.b64encode(file_content).decode('utf-8')

    # Update the global desired state
    manager.desired_state.speed = speed
    manager.desired_state.raster_type = raster_type
    manager.desired_state.path = pixel_list
    manager.desired_state.raster_mask = encoded_utf8

    # Broadcast once to the robot
    # This sends one single json packet containing all updated fields
    print(f"Bundled Execution: Broadcasting {len(pixel_list)} pixels at speed {speed}")
    await manager.broadcast_to_group(group=manager.robot_connections, state=manager.desired_state)

    return {
        "status": "success",
        "message": "Full execution packet dispatched to robot",
        "pixel_count": len(pixel_list)
    }

@app.post("/api/view_settings")
async def submit_settings(request: Request):
    data = await request.json()
    isTransformedViewOn = data.get("isTransformedViewOn")
    isThermalViewOn = data.get("isThermalViewOn")
    print(data)
    # isTransformedViewOn: bool = None
    # isThermalViewOn: bool = None
    
    manager.desired_state.isTransformedViewOn = isTransformedViewOn
    manager.desired_state.isThermalViewOn = isThermalViewOn

    print(f"Received settings: tranformed view on? {isTransformedViewOn}, thermal view on? {isThermalViewOn}")
    print("BROADCASTING")
    await manager.broadcast_to_group(group=manager.robot_connections, state=manager.desired_state)
    
    return {
        "status": "success",
        "isTransformedViewOn": isTransformedViewOn,
        "isThermalViewOn": isThermalViewOn,
        "message": "Settings dispatched to robot"
    }

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