import json
import numpy as np
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

    @staticmethod
    def from_pose(pose: tuple):
        x = pose[0]
        y = pose[1]
        z = pose[2]
        return RobotSchema(x=x, y=y, z=z)
    
    def to_mat(self):
        mat = np.identity(4)
        mat[0,3] = self.x
        mat[1,3] = self.y
        mat[2,3] = self.z
        return mat

    def to_str(self) -> str:
        data = {k: v for k, v in asdict(self).items() if v is not None}
        return json.dumps(data)

    def update(self, data: dict):
        for k, v in data.items():
            if(hasattr(self, k) and v is not None):
                self.__setattr__(k, v)
        return self