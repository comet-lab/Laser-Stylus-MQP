# This script scans a surface of a desired object and saves the required data for surface reconstruction
## Path setup and Imports
import time, cv2
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

import sys, pathlib
HERE = pathlib.Path(__file__).resolve().parent
# If this file is inside a subfolder, try parent too; stop at the first match.
for candidate in (HERE, HERE.parent, HERE.parent.parent):
    if (candidate / "robot").is_dir() and (candidate / "cameras").is_dir() and (candidate / "laser_control").is_dir():
        sys.path.insert(0, str(candidate))
        break    
from registration.cameraRegistration import Camera_Registration
from registration.transformations.depth_estimation import DepthEstimation
from registration.transformations.roi_selector import ROISelector

import matplotlib.pyplot as plt
from robot.robot_controller import Robot_Controller
from cameras.thermal_cam import ThermalCam
from cameras.RGBD_cam import RGBD_Cam
from laser_control.laser_arduino import Laser_Arduino


def exp_depth_scan(cam_reg, gridShape = np.array([15, 15]), squareSize = 0.002):
    input(f"Press Enter to continue depth estimation creation.")
    
    xPoints = (np.arange(gridShape[1]) - (gridShape[1] - 1) / 2) * squareSize
    yPoints = (np.arange(gridShape[0]) - (gridShape[0] - 1) / 2) * squareSize
    xValues, yValues = np.meshgrid(xPoints, yPoints)
    
    
    robot_path = np.hstack((xValues.reshape((-1, 1)), 
                            yValues.reshape((-1, 1)), 
                            np.full((xValues.size, 1), 0)))
    
    start_pos = robot_path[0, :]
    start_pose = np.eye(4)
    start_pose[:3, -1] = start_pos
    print("Heading to starting location")
    cam_reg.robot_controller.go_to_pose(start_pose @ cam_reg.robot_controller.home_pose)
    
    traj = cam_reg.robot_controller.create_custom_trajectory(robot_path, 0.01)
    depth, meta = cam_reg.scan_region_for_depth(traj)
    
    save_location = cam_reg.directory + cam_reg.calibration_folder + "/depth_map.npz"
    DepthEstimation.save_depth_npz(save_location, depth, meta)
    return depth, meta

def main():
    ##################################################################################
    #-------------------------------- Laser Config ----------------------------------#
    ##################################################################################
    
    laser_controller = Laser_Arduino()  # controls whether laser is on or off
    laser_on = False
    laser_controller.vf_valid_flag = True
    laser_controller.set_output(laser_on)
    
    ##################################################################################
    #------------------------------ Robot Config ------------------------------------#
    ##################################################################################
    # Create FrankaNode object for controlling robot
    robot_controller = Robot_Controller(laser_controller)
    # TODO fix running in container
    home_pose = robot_controller.load_home_pose()
    # home_pose = robot_controller.load_edit_pose()
    start_pos = np.array([0,0,0.10]) # [m,m,m]
    target_pose = np.array([[1.0, 0, 0, start_pos[0]],
                            [0,1,0,start_pos[1]],
                            [0,0,1,start_pos[2]],
                            [0,0,0,1]])
    robot_controller.go_to_pose(target_pose@home_pose,1) # Send robot to start position
    time.sleep(2)
    
    ##################################################################################
    #--------------------------- Thermal Cam Config ---------------------------------#
    ##################################################################################
    # Set up camera object
    window_scale = 1
    frame_rate = 50  
    temp_scale = 100.0  # based on temperature linear 10mK reading
    # start with full window so we can perform camera calibration. Additionally, set maximum frame rate at 50 hz, 
    # and the focal distance to 0.204 m. This seems to be at the right location to maximize the focal point around the 
    # free beam laser spot.
    therm_cam = None
    therm_cam = ThermalCam(IRFormat="TemperatureLinear10mK", height=int(480/window_scale),frame_rate="Rate50Hz",focal_distance=0.2) 
    
    ##################################################################################
    #------------------------------ RGBD Cam Config ---------------------------------#
    ##################################################################################
    rgbd_cam = RGBD_Cam() #Runs a thread internally
    
    ### -------------------- Scan Object for Surface Reconstruction ---------------- ####
    # # Create camera registration object to perform calibration and surface scan
    camera_reg = Camera_Registration(therm_cam,rgbd_cam, robot_controller, laser_controller)

    # # Define home pose of robot
    camera_reg.robot_controller.load_edit_pose()
    
    # Scan surface of desired object and save required data for surface reconstruction
    exp_depth_scan(camera_reg, gridShape = np.array([25, 25]), squareSize = 0.04/24)

    therm_cam.deinitialize_cam()

if __name__ == '__main__':
    main()