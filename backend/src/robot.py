#backend/src/robot.py

import json
from dataclasses import dataclass, asdict
from typing import List, Dict, Optional, Any

@dataclass
class RobotSchema:
    """
    Data Transfer Object representing the state of the Robot and UI.
    """
    #Coordinates (Real-time motion commands)
    x: Optional[float] = None
    y: Optional[float] = None
    z: Optional[float] = None
    rx: Optional[float] = None
    ry: Optional[float] = None
    rz: Optional[float] = None
    
    #Laser and Vision State
    laserX: Optional[float] = None
    laserY: Optional[float] = None
    averageHeat: Optional[float] = None
    maxHeat: Optional[float] = None
    beamWaist: Optional[float] = None
    
    #Flags & Views
    isLaserOn: Optional[bool] = None
    isRobotOn: Optional[bool] = None
    isLaserFiring: Optional[bool] = None
    isAutoHeightAdjustOn: Optional[bool] = None
    isRecordingOn: Optional[bool] = None
    isTransformedViewOn: Optional[bool] = None
    isThermalViewOn: Optional[bool] = None
    request_sync: Optional[bool] = None
    
    # 4. Settings (Parameters sent to Robot)
    speed: Optional[float] = None
    density: Optional[float] = None
    height: Optional[float] = None
    current_height: Optional[float] = None
    raster_type: Optional[str] = None
    pathEvent: Optional[str] = None
    
    #Complex Data and Masks (Transient)
    path: Optional[List[Dict[str, float]]] = None
    heat_markers: Optional[List[Dict[str, float]]] = None
    raster_mask: Optional[str] = None
    fixtures_mask: Optional[str] = None
    heat_mask: Optional[str] = None
    
    #Execution and Preview
    pathPrepared: Optional[bool] = None
    executeCommand: Optional[bool] = None
    path_preview: Optional[Dict[str, List[float]]] = None
    preview_duration: Optional[float] = None

    def flush(self) -> str:
        """
        Serializes the current state to JSON and resets transient fields 
        (masks, paths, markers) to None to prevent re-sending heavy data.
        """
        payload = self.to_str()
        
        # Reset transient fields
        self.raster_mask = None
        self.fixtures_mask = None
        self.heat_mask = None
        self.path = None
        self.heat_markers = None
        
        # Reset Execution/Preview Flags
        self.path_preview = None 
        self.preview_duration = None
        self.executeCommand = None
        self.request_sync = None
        
        # Clear motion commands (persist laserX and laserY)
        self.x = None
        self.y = None
        self.z = None
        self.rx = None
        self.ry = None
        self.rz = None
        
        return payload

    def to_str(self) -> str:
        """Converts non-None fields to a JSON string."""
        data = {k: v for k, v in asdict(self).items() if v is not None}
        return json.dumps(data)

    def to_dict(self) -> Dict[str, Any]:
        """Converts the schema to a dictionary."""
        return asdict(self)

    def update(self, data: Dict[str, Any]) -> None:
        """Updates fields based on a dictionary, ignoring None values."""
        for k, v in data.items():
            if hasattr(self, k) and v is not None:
                self.__setattr__(k, v)