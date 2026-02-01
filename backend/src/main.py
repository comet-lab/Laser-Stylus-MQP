from fastapi import FastAPI, HTTPException, WebSocket, UploadFile, Request, Form, File
from fastapi.middleware.cors import CORSMiddleware
import os
import base64
from robot import RobotSchema
from manager import ConnectionManager
import json
from typing import Optional # Import Optional

app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"], # changed to * to ensure IP access works
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

@app.post("/api/fixtures")
async def execute_fixtures(file: UploadFile = File(...)):
    # Process and save the fixtures mask
    file_content = await file.read()
    save_directory = "saved_masks"
    os.makedirs(save_directory, exist_ok=True)
    file_location = f"{save_directory}/{file.filename}"
    
    with open(file_location, "wb") as f:
        f.write(file_content)

    # Encode for websocket transmission
    encoded_utf8 = base64.b64encode(file_content).decode('utf-8')

    # Update the global desired state
    manager.desired_state.fixtures_mask = encoded_utf8

    print(f"Fixtures Update: Broadcasting mask")
    await manager.broadcast_to_group(group=manager.robot_connections, state=manager.desired_state)

    return {
        "status": "success",
        "message": "Fixtures mask dispatched to robot"
    }

# --- CORRECTED ENDPOINT ---
@app.post("/api/execute")
async def execute_bundled_command(
    speed: float = Form(...),
    raster_type: str = Form(None),
    density: float = Form(...),
    pixels: str = Form(...),
    is_fill: bool = Form(...), # NEW: Capture the boolean sent from frontend
    file: Optional[UploadFile] = File(None) # CHANGED: Defaults to None, making it optional
):
    try:
        pixel_list = json.loads(pixels)
    except json.JSONDecodeError:
        raise HTTPException(status_code=400, detail="Invalid pixel data format")

    encoded_utf8 = None

    # Only process the file if it was actually sent
    if file:
        file_content = await file.read()
        save_directory = "saved_masks"
        os.makedirs(save_directory, exist_ok=True)
        file_location = f"{save_directory}/{file.filename}"
        
        with open(file_location, "wb") as f:
            f.write(file_content)

        encoded_utf8 = base64.b64encode(file_content).decode('utf-8')

    manager.desired_state.speed = speed
    manager.desired_state.raster_type = raster_type if encoded_utf8 is not None else None
    manager.desired_state.density = density
    manager.desired_state.path = pixel_list
    # If no file, we explicitly set the mask to None or empty so the robot knows not to raster
    manager.desired_state.raster_mask = encoded_utf8 

    print(f"Bundled Execution: Broadcasting {len(pixel_list)} pixels. Fill: {is_fill}")
    await manager.broadcast_to_group(group=manager.robot_connections, state=manager.desired_state)

    return {
        "status": "success",
        "message": "Full execution packet dispatched to robot",
        "pixel_count": len(pixel_list),
        "has_raster": encoded_utf8 is not None
    }

@app.post("/api/view_settings")
async def submit_settings(request: Request):
    data = await request.json()
    isTransformedViewOn = data.get("isTransformedViewOn")
    isThermalViewOn = data.get("isThermalViewOn")
    
    manager.desired_state.isTransformedViewOn = isTransformedViewOn
    manager.desired_state.isThermalViewOn = isThermalViewOn

    await manager.broadcast_to_group(group=manager.robot_connections, state=manager.desired_state)
    
    return {
        "status": "success",
        "isTransformedViewOn": isTransformedViewOn,
        "isThermalViewOn": isThermalViewOn,
        "message": "Settings dispatched to robot"
    }

@app.post("/api/heat_markers")
async def submit_heat_markers(markers: str = Form(...)):
    try:
        markers_list = json.loads(markers)
    except json.JSONDecodeError:
        raise HTTPException(status_code=400, detail="Invalid markers format")

    manager.desired_state.heat_markers = markers_list
    
    # Broadcast to robot so it knows where to measure
    await manager.broadcast_to_group(
        group=manager.robot_connections,
        state=manager.desired_state
    )
    return {
        "status": "success",
        "marker_count": len(markers_list),
        "markers": markers_list,
        "message": "Heat markers dispatched to robot"
    }

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

@app.get("/current-coordinates")
def get_current_coordinates():
    return manager.current_state.to_str()