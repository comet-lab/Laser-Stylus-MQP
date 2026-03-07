import cv2, os 
import numpy as np
import matplotlib.pyplot as plt 
from typing import List, Dict
from dataclasses import dataclass


if __name__=='__main__':
    import sys, pathlib
    HERE = pathlib.Path(__file__).resolve().parent
    # If this file is inside a subfolder, try parent too; stop at the first match.
    for candidate in (HERE, HERE.parent, HERE.parent.parent):
        if (candidate / "robot").is_dir() and (candidate / "cameras").is_dir() and (candidate / "laser_control").is_dir():
            sys.path.insert(0, str(candidate))
            break
        
from backend.datastorage import SystemDataStore, CaliState
from common.vision import normalize_homography
from registration.transformations.depth_estimation import DepthEstimation


@dataclass(frozen=True)
class homography_stack:
    H_rw: Dict[float, np.ndarray]
    H_tw : Dict[float, np.ndarray]
    H_rt : Dict[float, np.ndarray]
    H_wr: Dict[float, np.ndarray]
    H_wt : Dict[float, np.ndarray]

def read_file():
    directory = os.getcwd() 
    calibration_folder = r"surgical_system\py_src\image_developement\Calibration Data\calibration_full\20260306\run_001\cali.pkl"
    full_path = os.path.join(directory, calibration_folder)
    print("Loading Path: ", full_path)
    
    data_set: List[CaliState] = []
    data_set = SystemDataStore.load_storage(full_path) # list of cali_states
    
    H_rgb_to_world = {}
    H_therm_to_world = {} 
    H_rgb_to_therm = {}
    
    H_world_to_rgb = {}
    H_world_to_therm = {} 
    
    for cali in data_set:
        height = float(cali.cali_type)
        world_points = cali.payload["obj_Points"]
        rgb_points = cali.payload["rgb_img_points"].T
        therm_points = cali.payload["rgb_img_points"].T
        
        H_rgb_to_world[height] = cv2.findHomography(rgb_points, world_points)[0]
        H_therm_to_world[height] = cv2.findHomography(therm_points, world_points)[0]
        H_rgb_to_therm[height] = cv2.findHomography(rgb_points, therm_points)[0]
        
        H_world_to_rgb[height] = np.linalg.inv(H_rgb_to_world[height])
        H_world_to_therm[height] = np.linalg.inv(H_therm_to_world[height])
        
    return homography_stack(H_rw = DepthEstimation.create_dense_stack(H_rgb_to_world), 
                            H_tw = DepthEstimation.create_dense_stack(H_therm_to_world), 
                            H_rt = DepthEstimation.create_dense_stack(H_rgb_to_therm),
                            H_wr = DepthEstimation.create_dense_stack(H_world_to_rgb),
                            H_wt = DepthEstimation.create_dense_stack(H_world_to_therm))

# "therm_img_points"
# "rgb_img_points"
# "obj_Points" # m 
# "grid shape"
# "square size"
# "boundary" # m 


def simulate_scan(h_stack: homography_stack, cmd_points, obs_points):
    num_points = len(cmd_points)
    z_best, err_best, uv_best, conf = DepthEstimation.estimate_depth_from_dense_stack(
            h_stack.H_rw,
            obs_uvs=obs_points,
            cmd_XYs=cmd_points,
            refine=True) 
    
    world_points = np.hstack((cmd_points, z_best[:, None]))
    depth_map, meta = DepthEstimation.generate_depth_mapping(world_points)
    depth_map = DepthEstimation.patch_depth(depth_map)
    DepthEstimation.plot_depth_surface(depth_map, meta)
    
    # for i in range(num_points):
    #     z_best, err_best, uv_best, conf = DepthEstimation.estimate_depth_from_dense_stack(
    #         h_stack.H_rw,
    #         obs_uv=obs_points[i],
    #         cmd_XY=cmd_points[i],
    #         refine=True) 
    #     print(z_best)
    
    # pass
    


def main():
    scanned_points = np.load("surgical_system/py_src/image_developement/Calibration Data/scan_points.npz", allow_pickle=True)
    cmd_points, obs_points = scanned_points["cmd_points"], scanned_points["obs_pixels"]
    h_stack: homography_stack = read_file()
    simulate_scan(h_stack, cmd_points, obs_points)
    
    
    
    
    

if __name__ == "__main__":
    main()