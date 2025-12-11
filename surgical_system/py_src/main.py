import os, time, sys, warnings
import json
import cv2
import base64
import numpy as np
import asyncio
import websockets
import time
from datetime import datetime as dt
from scipy.spatial.transform import Rotation
from robot.robot import RobotSchema
from dataclasses import dataclass, asdict
import matplotlib.pyplot as plt

from robot.motion_planning import Motion_Planner
# Classes
mock_robot = os.getenv("MOCK_ROBOT", "0") == "1"
print(f"Mocking? {mock_robot}")
if(not mock_robot):
    from robot.robot_controller import Robot_Controller
    from cameras.thermal_cam import ThermalCam
    from cameras.RGBD_cam import RGBD_Cam
    from laser_control.laser_arduino import Laser_Arduino
    from registration.cameraRegistration import Camera_Registration
else:
    from robot.mock_robot_controller import MockRobotController
    from cameras.mock_camera import MockCamera
    from registration.mock_camera_registration import MockCameraRegistration
    from laser_control.mock_laser import MockLaser

from cameras.broadcast import Broadcast
from backend.listener import BackendConnection
from backend.handler import Handler


async def main():
    camera_calibration = False
    
            
    
    ##################################################################################
    #------------------------------ Robot Config ------------------------------------#
    ##################################################################################
    # Create FrankaNode object for controlling robot
    robot_controller = Robot_Controller() if not mock_robot else MockRobotController()
    home_pose = robot_controller.load_home_pose()
    start_pos = np.array([0,0,0.1]) # [m,m,m]
    start_pose = np.array([[1.0, 0, 0, start_pos[0]],
                            [0,1,0,start_pos[1]],
                            [0,0,1,start_pos[2]],
                            [0,0,0,1]])
    robot_controller.go_to_pose(start_pose@home_pose,1) # Send robot to start position

    desired_state = RobotSchema()
    await asyncio.sleep(2) # ?
    
    ##################################################################################
    #----------------------------------- Cam Config ---------------------------------#
    ##################################################################################
    # Set up camera object
    window_scale = 1
    frame_rate = 50  
    temp_scale = 100.0  # based on temperature linear 10mK reading
    # start with full window so we can perform camera calibration. Additionally, set maximum frame rate at 50 hz, 
    # and the focal distance to 0.204 m. This seems to be at the right location to maximize the focal point around the 
    # free beam laser spot.
    therm_cam = None
    rgbd_cam = None
    
    if(not mock_robot):
        therm_cam = ThermalCam(IRFormat="TemperatureLinear10mK", height=int(480/window_scale),frame_rate="Rate50Hz",focal_distance=0.2)
        rgbd_cam = RGBD_Cam() #Runs a thread internally
        rgbd_cam.set_default_setting() # Auto-exposure
    else:
        therm_cam = MockCamera(cam_type="thermal")
        rgbd_cam = MockCamera(cam_type="color")
        

    
    ##################################################################################
    #-------------------------------- Laser Config ----------------------------------#
    ##################################################################################
    
    laser_obj = None
    if(not mock_robot):
        laser_obj = Laser_Arduino()  # controls whether laser is on or off
        laser_obj.set_output(False)
    else:
        laser_obj = MockLaser()
        
    # TODO mock camera registration object
    camera_reg = None
    if(not mock_robot):
        camera_reg = Camera_Registration(therm_cam, rgbd_cam, robot_controller, laser_obj)
    else:
        camera_reg = MockCameraRegistration(therm_cam, rgbd_cam, robot_controller, laser_obj)
    print("Starting Streams ")
    b = Broadcast(mocking=mock_robot)
    print(f"Broadcast connection status: {b.connect()}")
    
    # def camera_broadcast_fn():
    #     while True:
    #         latest = camera_reg.get_cam_latest(cam_type=cam_type)
    #         if(type(camera_broadcast_fn.old_frame) != type(None)):
    #             if(np.sum(cv2.absdiff(latest, camera_broadcast_fn.old_frame)) == 0):
    #                 # Same frame
    #                 continue
    #         if isinstance(latest, dict):
    #             latest = latest.get(cam_type, None)
    #         if(type(latest) == type(None)):
    #             # No new frame
    #             continue
    #         if(b.connected):
    #             b.publish_frame(latest)
    #         else:
    #             b.connect()
    #             print('connecting...')
    #             time.sleep(2)
    # camera_broadcast_fn.old_frame = None
    # asyncio.create_task(camera_broadcast_fn())
    

    print("Cameras are ready")
    
    ##################################################################################
    #----------------------------- Backend Connection -------------------------------#
    ##################################################################################

    control_flow_handler = Handler(
        desired_state=desired_state,
        robot_controller=robot_controller,
        cam_reg=camera_reg,
        laser_obj=laser_obj,
        start_pose=start_pose,
        mock_robot=mock_robot
    )
    
    ##################################################################################
    #----------------------------- Camera Calibration -------------------------------#
    ##################################################################################
    
    
    if camera_calibration:
        pass

    start_pose[2,3] = 0.0
    robot_controller.go_to_pose(start_pose@home_pose,1) # Send robot to start position

    while (True):
        await control_flow_handler.main_loop()            
            
        # Camera frame publishing
        latest = camera_reg.get_cam_latest(cam_type=control_flow_handler.cam_type)
        
        if control_flow_handler.desired_state.isTransformedViewOn:
            latest = camera_reg.get_transformed_view(latest, cam_type=control_flow_handler.cam_type)
            latest = cv2.resize(latest, (1280, 720))

        if isinstance(latest, dict):
            latest = latest.get(control_flow_handler.cam_type, None)
        if(type(latest) == type(None)):
            continue
        if(b.connected):
            b.publish_frame(latest)
        else:
            b.connect()
            print('connecting...')
            time.sleep(2)
  

if __name__ == "__main__":
    asyncio.run(main())