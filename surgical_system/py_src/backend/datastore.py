from __future__ import annotations
from dataclasses import dataclass, field
from typing import Optional, Dict, Any, Tuple
import threading
import numpy as np


@dataclass(frozen=True)
class CameraFrame:
    t: float
    sensor: str                   # "rgb", "depth", "thermal"
    img: np.ndarray               # HxWxC or HxW
    frame_id: str                 # "camera_rgb", ...
    meta: Dict[str, Any] = field(default_factory=dict)

@dataclass(frozen=True)
class RobotState:
    t: float
    q: np.ndarray                 # (7,)
    dq: np.ndarray                # (7,)
    frame_id: str = "world"

@dataclass(frozen=True)
class UserCommand:
    t: float
    mode: str                     # "draw", "live_control", 
    payload: Dict[str, Any]       # markers, path, etc.

@dataclass(frozen=True)
class Transform:
    t: float
    parent: str
    child: str
    T_parent_child: np.ndarray    # (4,4)
    cov: Optional[np.ndarray] = None
    
class SystemDataStore:
    """
    Central in-memory data store:
      - Latest camera frames
      - Latest robot state
      - Latest user command
      - Transform tree / calibration artifacts
    """
    def __init__(self):
        self.camera_frames: Dict[str, Any] = {
                                    "rgb": [],
                                    "depth": [],
                                    "thermal": [],
                                }
        self.robot_latest = []
        self.user_latest = []

        # self.transforms_latest: Dict[Tuple[str, str], LatestValue] = {}  # (parent,child) -> latest

        # calibration artifacts (homography stacks, intrinsics, etc.)
        self.calib: Dict[str, Any] = {}
        self._calib_lock = threading.Lock()

    # --- Camera ---
    def put_camera(self, frame: CameraFrame):
        self.camera_hist[frame.sensor].push(frame)

    def get_camera_latest(self, sensor: str) -> Optional[CameraFrame]:
        return self.camera_latest[sensor].get()

    # --- Robot ---
    def put_robot(self, st: RobotState):
        self.robot_hist.push(st)

    def get_robot_latest(self) -> Optional[RobotState]:
        return self.robot_latest.get()

    # --- User ---
    def put_user(self, cmd: UserCommand):
        self.user_hist.push(cmd)

    def get_user_latest(self) -> Optional[UserCommand]:
        return self.user_latest.get()

    # --- Transforms ---
    # def put_transform(self, tf: Transform):
    #     key = (tf.parent, tf.child)
    #     if key not in self.transforms_latest:
    #         self.transforms_latest[key] = LatestValue()
    #     self.transforms_latest[key].set(tf)

    # def get_transform_latest(self, parent: str, child: str) -> Optional[Transform]:
    #     lv = self.transforms_latest.get((parent, child))
    #     return lv.get() if lv else None

    # --- Calibration ---
    def set_calib(self, key: str, value: Any):
        self.calib[key] = value

    def get_calib(self, key: str, default=None):
        return self.calib.get(key, default)