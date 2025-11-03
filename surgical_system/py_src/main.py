import os, time, sys, warnings
import numpy as np
import asyncio
import websockets
from scipy.spatial.transform import Rotation

# Classes
from robot.robot_controller import Robot_Controller
from cameras.thermal_cam import ThermalCam
from cameras.RGBD_cam import RGBD_Cam
from laser_control.laser_arduino import Laser_Arduino

from robot.mock_franka_client import MockFrankaClient
mock_robot = os.getenv("MOCK_ROBOT", "false") == "true"
print(f"Mocking? {mock_robot}")
if(not mock_robot):
    from cameras.thermal_cam import ThermalCam
from backend.listener import BackendConnection


async def main():
    pathToCWD = os.getcwd()
    camera_calibration = False
    
    ##################################################################################
    #------------------------------ Robot Config ------------------------------------#
    ##################################################################################
    # Create FrankaNode object for controlling robot
    robot_controller = Robot_Controller() if not mock_robot else MockFrankaClient()
    # TODO fix running in container
    home_pose = robot_controller.load_home_pose(home_pose_path="home_pose.csv")
    start_pos = np.array([0,0,0.35]) # [m,m,m]
    target_pose = np.array([[1.0, 0, 0, start_pos[0]],
                            [0,1,0,start_pos[1]],
                            [0,0,1,start_pos[2]],
                            [0,0,0,1]])
    robot_controller.goToPose(target_pose@home_pose,1) # Send robot to start position
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
    if(not mock_robot):
        therm_cam = ThermalCam(IRFormat="TemperatureLinear10mK", height=int(480/window_scale),frameRate="Rate50Hz",focalDistance=0.2) 
    
    ##################################################################################
    #------------------------------ RGBD Cam Config ---------------------------------#
    ##################################################################################
    rgbd_cam = RGBD_Cam() #Runs a thread internally

    
    ##################################################################################
    #-------------------------------- Laser Config ----------------------------------#
    ##################################################################################
    
    laser_obj = Laser_Arduino()  # controls whether laser is on or off
    laser_on = False
    laser_obj.set_output(laser_on)

    
    ##################################################################################
    #----------------------------- Backend Connection -------------------------------#
    ##################################################################################
    
    backend_connection = BackendConnection(
        send_fn=lambda: "Hello from robot!",
        recv_fn=lambda msg: print("Server sent: " + msg)
    )
    await backend_connection.connect_to_websocket()
    
    ##################################################################################
    #----------------------------- Camera Calibration -------------------------------#
    ##################################################################################
    
    if camera_calibration:
        pass

    await asyncio.Future()
    
    # while(not therm_cam.is_ready() or not rgbd_cam.is_ready()):
    #     time.sleep(2)
        # print("Waiting for: Therm ",therm_cam.isReady(), " RGBD ", rgbd_cam.isReady())
    
    print("Cameras are ready")
    therm_cam.start_stream() # Start camera stream
    rgbd_cam.start_stream()
    print("Starting Streams ")
    while(True):
        rgbd_cam.display_all_streams() #Test Streaming
        therm_cam.display()
        
    
    therm_cam.deinitialize_cam()

    
    

if __name__ == "__main__":
    asyncio.run(main())