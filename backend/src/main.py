#backend/src/main.py

from fastapi import FastAPI, HTTPException, WebSocket, UploadFile, Request, Form, File
from fastapi.middleware.cors import CORSMiddleware
import os
import base64
from robot import RobotSchema
from manager import ConnectionManager
import json
from typing import Optional
import random
import math
from typing import Optional, List, Dict, Any

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
    return {"stream_url": "http://169.254.0.3:8889/mystream"}

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

# ============================================================================
#  PATH GENERATION HELPERS
# ============================================================================

def generate_fake_raster_path(pixel_list: List[Dict[str, float]], density: float) -> List[Dict[str, float]]:
    """
    Fallback simulation: creates a zig-zag pattern within the bounding box.
    """
    if not pixel_list: return []

    xs = [p['x'] for p in pixel_list]
    ys = [p['y'] for p in pixel_list]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)

    generated_path = []
    
    step = max(10.0, 200.0 / (density if density > 0 else 1)) 
    
    current_y = min_y
    going_right = True
    point_resolution = 10.0 

    while current_y <= max_y:
        start_x = min_x if going_right else max_x
        end_x = max_x if going_right else min_x
        
        dist = abs(end_x - start_x)
        num_sub_steps = int(dist / point_resolution)
        direction = 1 if end_x > start_x else -1
        
        if num_sub_steps < 1:
            generated_path.append({"x": start_x, "y": current_y})
            generated_path.append({"x": end_x, "y": current_y})
        else:
            for i in range(num_sub_steps + 1):
                x_pos = start_x + (i * point_resolution * direction)
                if direction > 0: x_pos = min(x_pos, end_x)
                else:             x_pos = max(x_pos, end_x)
                generated_path.append({"x": x_pos, "y": current_y})

        current_y += step
        going_right = not going_right
        
    return generated_path

# --- PREVIEW ENDPOINT ---
@app.post("/api/preview")
async def preview_path(
    speed: float = Form(...),
    raster_type: str = Form(None),
    density: float = Form(...),
    pixels: str = Form(...),
    is_fill: bool = Form(...),
    file: Optional[UploadFile] = File(None)
):
    """
    Preview endpoint now sends all execution data to the robot for path computation.
    The robot will compute the path and send it back via websocket.
    """
    try:
        pixel_list = json.loads(pixels)
    except json.JSONDecodeError:
        raise HTTPException(status_code=400, detail="Invalid pixel data format")

    encoded_utf8 = None

    # Process the file if it was sent (for raster fills)
    if file:
        file_content = await file.read()
        save_directory = "saved_masks"
        os.makedirs(save_directory, exist_ok=True)
        file_location = f"{save_directory}/{file.filename}"
        
        with open(file_location, "wb") as f:
            f.write(file_content)

        encoded_utf8 = base64.b64encode(file_content).decode('utf-8')

    # Update desired state with ALL path data
    manager.desired_state.speed = speed
    manager.desired_state.raster_type = raster_type if encoded_utf8 is not None else None
    manager.desired_state.density = density
    manager.desired_state.path = pixel_list
    manager.desired_state.raster_mask = encoded_utf8 if encoded_utf8 is not None else None
    manager.desired_state.pathPrepared = True  # Mark as prepared
    manager.desired_state.executeCommand = False  # Not executing yet

    print(f"Preview: Broadcasting {len(pixel_list)} pixels for path computation. Fill: {is_fill}")
    await manager.broadcast_to_group(group=manager.robot_connections, state=manager.desired_state)

    # --- REAL MODE (Default): Return empty path, Frontend waits for WS ---
    return {
        "status": "pending",
        "duration": 0,
        "path": [], 
        "message": "Computation initiated on Robot"
    }

    # --- FAKE MODE (Uncomment for testing without Robot) ---
    final_path = []
    duration = 0

    if is_fill:
        final_path = generate_fake_raster_path(pixel_list, density)
    else:
        final_path = pixel_list[::5]

    total_distance = 0
    if len(final_path) > 1:
        for i in range(1, len(final_path)):
            dx = final_path[i]['x'] - final_path[i-1]['x']
            dy = final_path[i]['y'] - final_path[i-1]['y']
            total_distance += math.sqrt(dx*dx + dy*dy)

    fake_speed_factor = speed * 10
    if fake_speed_factor == 0: fake_speed_factor = 1
    duration = (total_distance / fake_speed_factor) + random.uniform(0.5, 1.5)

    return {
        "status": "success",
        "duration": duration,
        "path": final_path,
        "message": "Preview data sent to robot (Simulation)"
    }
    
# --- EXECUTION ENDPOINT ---
@app.post("/api/execute")
async def execute_path():
    """
    Execute endpoint now only sends the execute command.
    All path data should already be prepared via the preview endpoint.
    """
    if not manager.desired_state.pathPrepared:
        raise HTTPException(
            status_code=400, 
            detail="Path not prepared. Please preview the path first."
        )

    # Set execute command flag
    manager.desired_state.executeCommand = True

    print(f"Execute: Sending start command to robot")
    await manager.broadcast_to_group(group=manager.robot_connections, state=manager.desired_state)

    return {
        "status": "success",
        "message": "Execute command dispatched to robot"
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
    
@app.post("/api/heat_area")
async def update_heat_area(file: UploadFile = File(...)):
    file_content = await file.read()

    # Save specifically as heatarea.png
    save_directory = "saved_masks"
    os.makedirs(save_directory, exist_ok=True)
    file_location = f"{save_directory}/heatarea.png"
    
    with open(file_location, "wb") as f:
        f.write(file_content)

    encoded_utf8 = base64.b64encode(file_content).decode('utf-8')
    manager.desired_state.heat_mask = encoded_utf8
    
    print("Heat Area Update: Broadcasting mask")
    await manager.broadcast_to_group(group=manager.robot_connections, state=manager.desired_state)
    
    return {
        "status": "success", 
        "message": "Heat area mask saved and dispatched"
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