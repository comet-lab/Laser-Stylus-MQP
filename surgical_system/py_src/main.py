import os, time, subprocess, sys, warnings
import numpy as np
from scipy.spatial.transform import Rotation

# Classes
from robot.franka_client import FrankaClient
from calibration.Utilities_functions import loadHomePose
from cameras.thermal_cam import ThermalCam
# RGBD camera
from laser_control.laser_arduino import Laser_Arduino

def main():
    pathToCWD = os.getcwd()
    camera_calibration = False
    
    ##################################################################################
    #------------------------------ Robot Config ------------------------------------#
    ##################################################################################
    # Create FrankaNode object for controlling robot
    robot_obj = FrankaClient()  
    subprocess.Popen([pathToCWD + "/cpp_src/main"])
    home_pose = loadHomePose(home_pose_path="home_pose.csv")
    start_pos = np.array([0,0,0.35]) # [m,m,m]
    target_pose = np.array([[1.0, 0, 0, start_pos[0]],
                            [0,1,0,start_pos[1]],
                            [0,0,1,start_pos[2]],
                            [0,0,0,1]])
    robot_obj.send_pose(target_pose@home_pose,1) # Send robot to start position
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
    cam_obj = ThermalCam(IRFormat="TemperatureLinear10mK", height=int(480/window_scale),frameRate="Rate50Hz",focalDistance=0.2) 
    cam_obj.set_acquisition_mode()
    
    ##################################################################################
    #------------------------------ RGBD Cam Config ---------------------------------#
    ##################################################################################
    
    # Empty 
    
    ##################################################################################
    #-------------------------------- Laser Config ----------------------------------#
    ##################################################################################
    try:
        laser_obj = Laser_Arduino()  # controls whether laser is on or off
        laser_on = False
    except:
        print("Failed to connect to Laser...")
    
    ##################################################################################
    #----------------------------- Backend Connection -------------------------------#
    ##################################################################################
    
    # Empty 
    
    backend_connection = False
    
    ##################################################################################
    #----------------------------- Camera Calibration -------------------------------#
    ##################################################################################
    
    if camera_calibration:
        pass
    
    
    
    
    

if __name__=='__main__':
    main()