import json
from dataclasses import dataclass, asdict

@dataclass
class RobotSchema:
    x: float = None
    y: float = None
    z: float = None
    rx: float = None
    ry: float = None
    rz: float = None
    laserX: float = None
    laserY: float = None
    averageHeat: float = None
    beamWaist: float = None
    isLaserOn: bool = None
    isRobotOn: bool = None
    isTransformedViewOn: bool = None
    isThermalViewOn: bool = None
    pathEvent: str = None
    raster_mask: str = None
    fixtures_mask: str = None
    raster_type: str = None
    speed: float = None
    path: list[dict[str, float]] = None
    # heat_markers: List of dicts, potentially containing x, y, and temp
    heat_markers: list[dict[str, float]] = None

    def flush(self):
            payload = self.to_str()
            self.raster_mask = None
            self.fixtures_mask = None
            self.path = None
            # Do NOT clear heat_markers here if you want them to persist for temperature updates
            # However, usually we want to clear commands. 
            # If the backend treats this as a command to "set markers", clearing is fine.
            # If the backend treats this as "current marker state with temps", we might keep it.
            # For this logic, we'll clear it to avoid re-sending large payloads if unchanged.
            self.heat_markers = None 
            return payload

    def to_str(self) -> str:
        data = {k: v for k, v in asdict(self).items() if v is not None}
        return json.dumps(data)

    def to_dict(self) -> dict:
        return asdict(self)

    def update(self, data: dict):
        for k, v in data.items():
            if hasattr(self, k) and v is not None:
                self.__setattr__(k, v)