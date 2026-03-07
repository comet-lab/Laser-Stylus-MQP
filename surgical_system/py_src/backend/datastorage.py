from __future__ import annotations
from dataclasses import dataclass
from typing import Dict, Any
import os, pickle, json
from datetime import datetime
import numpy as np


@dataclass(frozen=True)
class CameraFrame:
    t: float
    sensor: str                   # "rgb", "depth", "thermal"
    img: np.ndarray               # HxWxC or HxW
    frame_id: str                 # "camera_rgb", ...

@dataclass(frozen=True)
class RobotState:
    t: float
    q: np.ndarray                 # (7,)
    dq: np.ndarray                # (7,)
    laser_on: bool 
    frame_id: str = "world"
    

@dataclass(frozen=True)
class UserCommand:
    t: float
    mode: str                     # "draw", "live_control", 
    payload: Dict[str, Any]       # markers, path, etc.
    
@dataclass(frozen=True)
class CaliState:
    cali_type: str                     
    payload: Dict[str, Any]       
    
class SystemDataStore:
    """
    Central in-memory data store:
      - Latest camera frames
      - Latest robot state
      - Latest user command
      - Transform tree / calibration artifacts
    """
    def __init__(self, home_pose: np.ndarray, homography: Dict[str, np.ndarray]):
        self.camera_history: Dict[str, Any] = {
                                    "rgb": [],
                                    "depth": [],
                                    "thermal": []}
        
        self.robot_history: list[RobotState] = []
        self.user_history:list[UserCommand] = []
        
        self.transforms : Dict[str, np.ndarray] = {"home": home_pose,
                                           "rgb_w": homography["rgb_w"],
                                           "therm_w": homography["therm_w"],
                                           "rgb_therm": homography["rgb_therm"]}

        # calibration artifacts (homography stacks, intrinsics, etc.)
        self.homography_stack: Dict[str, Any] = {"rgb_w_stack": homography["rgb_w_stack"],
                                           "therm_w_stack": homography["therm_w_stack"],
                                           "rgb_therm_stack": homography["rgb_therm_stack"]}
        


    # --- Camera ---
    def put_camera(self, sensor: str, t: float, img: np.ndarray, frame_id: str):
        if sensor in self.camera_history:
            self.camera_history[sensor].append(CameraFrame(
                t = t,
                sensor = sensor,                  
                img = img,               
                frame_id = frame_id ))


    # --- Robot ---
    def put_robot(self, q:np.ndarray, dq: np.ndarray, t: float, laser_on: bool):
        self.robot_history.append(RobotState(
            q=np.asarray(q, float),
            dq=np.asarray(dq, float),
            t=t,
            laser_on=laser_on))

    # --- User ---
    def put_user(self, t: float, mode: str, payload: Dict[str, Any]):
        self.user_history.append(UserCommand(
            t = t,
            mode = mode, # "draw", "live_control", 
            payload = payload))


    # --- Save and Load ---
    def to_state(self) -> dict:
        return {
            "schema_version": 1,
            "camera_history": self.camera_history,
            "robot_history": self.robot_history,
            "user_history": self.user_history,
            "transforms": self.transforms,
            "homography_stack": self.homography_stack,
        }
    
    def _next_index(self, folder: str, prefix: str) -> int:
        if not os.path.isdir(folder):
            return 1
        nums = []
        for d in os.listdir(folder):
            if d.startswith(prefix):
                try:
                    nums.append(int(d.split("_")[-1]))
                except ValueError:
                    pass
        return (max(nums) + 1) if nums else 1


    def save_data_storage(self, root: str, name: str = "run", reset: bool = False):
        os.makedirs(root, exist_ok=True)

        date_str = datetime.now().strftime("%Y%m%d")
        date_folder = os.path.join(root, date_str)
        os.makedirs(date_folder, exist_ok=True)

        run_idx = self._next_index(date_folder, name)
        run_folder = os.path.join(date_folder, f"{name}_{run_idx:03d}")
        os.makedirs(run_folder, exist_ok=False)

        # --- Save metadata ---
        meta = {
            "schema_version": 1,
            "date": date_str,
            "run_name": f"{name}_{run_idx:03d}",
            "sensors": list(self.camera_history.keys()),
            "counts": {
                "rgb": len(self.camera_history.get("rgb", [])),
                "depth": len(self.camera_history.get("depth", [])),
                "thermal": len(self.camera_history.get("thermal", [])),
                "robot": len(self.robot_history),
                "user": len(self.user_history),
            },
        }
        with open(os.path.join(run_folder, "meta.json"), "w") as f:
            json.dump(meta, f, indent=2)

        # --- Save transforms / calibration artifacts ---
        np.savez_compressed(
            os.path.join(run_folder, "calibration.npz"),
            **{k: np.asarray(v) for k, v in self.transforms.items()},
            **{k: np.asarray(v, dtype=object) for k, v in self.homography_stack.items()},
        )

        # --- Save robot/user histories ---
        if self.robot_history:
            t = np.array([r.t for r in self.robot_history], dtype=float)
            q = np.stack([r.q for r in self.robot_history]).astype(float)
            dq = np.stack([r.dq for r in self.robot_history]).astype(float)
            laser_on = np.array([r.laser_on for r in self.robot_history], dtype=bool)
            np.savez_compressed(os.path.join(run_folder, "robot.npz"), t=t, q=q, dq=dq, laser_on=laser_on)

        # User history can remain pickle unless you want strict JSON
        with open(os.path.join(run_folder, "user.pkl"), "wb") as f:
            pickle.dump(self.user_history, f)

        # Camera history:
        with open(os.path.join(run_folder, "camera.pkl"), "wb") as f:
            pickle.dump(self.camera_history, f)

        print(f"Saved run to: {run_folder}")

        if reset:
            self.camera_history = {"rgb": [], "depth": [], "thermal": []}
            self.robot_history = []
            self.user_history = []

        return run_folder
    
    @staticmethod
    def load_storage(file_path):
        try:
            with open(file_path, "rb") as f:
                return pickle.load(f)
        except Exception as e:
            print(f"[load_storage] Failed to open storage: {e}")
            raise
    

    # --- Transforms ---

    # --- Calibration ---
