import json
import numpy as np
from dataclasses import dataclass, asdict
from typing import List, Dict

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
    isRobotOn: bool = None
    isTransformedViewOn: bool = None
    isThermalViewOn: bool = None
    pathEvent: str = None
    raster_mask: str = None
    raster_type: str = None
    speed: float = None
    path: List[Dict[str, float]] = None


    @staticmethod
    def from_pose(pose: np.ndarray):
        x = pose[0,3]
        y = pose[1,3]
        z = pose[2,3]
        return RobotSchema(x=x, y=y, z=z)
    
    def _to_mat(self):
        mat = np.identity(4)
        mat[0,3] = self.x
        mat[1,3] = self.y
        mat[2,3] = self.z
        return mat
    
    def go_to_pose(self, home_t: np.ndarray, robot_controller):
        desired_task_pose = self._to_mat()
        desired_robot_pose = desired_task_pose@home_t
        return robot_controller.go_to_pose(desired_robot_pose)

    def to_str(self) -> str:
        data = {k: v for k, v in asdict(self).items() if v is not None}
        return json.dumps(data)

    def update(self, data: dict):
        for k, v in data.items():
            if(hasattr(self, k) and v is not None):
                self.__setattr__(k, v)
        return self