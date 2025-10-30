import os, time, sys, warnings, threading
import numpy as np
from scipy.spatial.transform import Rotation

# Classes
from robot.robot_controller import Robot_Controller
from cameras.thermal_cam import ThermalCam
from cameras.RGBD_cam import RGBD_Cam
from laser_control.laser_arduino import Laser_Arduino

def main():
    camera_calibration = False
    
    ##################################################################################
    #------------------------------ Robot Config ------------------------------------#
    ##################################################################################
    # Create FrankaNode object for controlling robot
    robot_controller = Robot_Controller()
    home_pose = robot_controller.loadHomePose()
    start_pos = np.array([0,0,0.3]) # [m,m,m]
    target_pose = np.array([[1.0, 0, 0, start_pos[0]],
                            [0,1,0,start_pos[1]],
                            [0,0,1,start_pos[2]],
                            [0,0,0,1]])
    robot_controller.goToPose(target_pose@home_pose,1) # Send robot to start position
    
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
    therm_cam = ThermalCam(IRFormat="TemperatureLinear10mK", height=int(480/window_scale),frameRate="Rate50Hz",focalDistance=0.2) 
    therm_cam.set_acquisition_mode()
    therm_cam.start_stream() # Start camera stream
    
    ##################################################################################
    #------------------------------ RGBD Cam Config ---------------------------------#
    ##################################################################################
    try:
        rgbd_cam = RGBD_Cam() #Runs a thread internally
    except:
        print("Error in connecting to RGB-D Camera")
    
    ##################################################################################
    #-------------------------------- Laser Config ----------------------------------#
    ##################################################################################
    try:
        laser_obj = Laser_Arduino()  # controls whether laser is on or off
        laser_on = False
        laser_obj.set_output(laser_on)
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
    
    rgbd_cam.display_all_streams() #Test Streaming
    
    

if __name__=='__main__':
    main()