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
    beamWaist: float = None
    isLaserOn: bool = None

    def to_str(self) -> str:
        data = {k: v for k, v in asdict(self).items() if v is not None}
        return json.dumps(data)

    def update(self, data: dict):
        for k, v in data.items():
            if(hasattr(self, k) and v is not None):
                self.__setattr__(k, v)