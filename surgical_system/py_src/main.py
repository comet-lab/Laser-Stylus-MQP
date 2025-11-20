import os, time, sys, warnings
import json
import cv2
import base64
import numpy as np
import asyncio
import websockets
import time
from scipy.spatial.transform import Rotation
from robot.robot import RobotSchema

# Classes
mock_robot = os.getenv("MOCK_ROBOT", "0") == "1"
print(f"Mocking? {mock_robot}")
if(not mock_robot):
    from robot.robot_controller import Robot_Controller
    from cameras.thermal_cam import ThermalCam
    from cameras.RGBD_cam import RGBD_Cam
    from laser_control.laser_arduino import Laser_Arduino
else:
    from robot.mock_franka_client import MockRobotController
    from cameras.mock_camera import MockCamera

from cameras.broadcast import Broadcast
from backend.listener import BackendConnection


async def main():
    camera_calibration = False
    
            
    
    ##################################################################################
    #------------------------------ Robot Config ------------------------------------#
    ##################################################################################
    # Create FrankaNode object for controlling robot
    robot_controller = Robot_Controller() if not mock_robot else MockRobotController()
    home_pose = robot_controller.load_home_pose()
    start_pos = np.array([0,0,0.35]) # [m,m,m]
    target_pose = np.array([[1.0, 0, 0, start_pos[0]],
                            [0,1,0,start_pos[1]],
                            [0,0,1,start_pos[2]],
                            [0,0,0,1]])
    robot_controller.go_to_pose(target_pose@home_pose,1) # Send robot to start position
    desired_state = RobotSchema()
    await asyncio.sleep(2)
    
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
        therm_cam = ThermalCam(IRFormat="TemperatureLinear10mK", height=int(480/window_scale),frame_rate="Rate50Hz",focal_distance=0.2)
    else:
        therm_cam = MockCamera()
    
    ##################################################################################
    #------------------------------ RGBD Cam Config ---------------------------------#
    ##################################################################################
    # rgbd_cam = RGBD_Cam() #Runs a thread internally

    
    ##################################################################################
    #-------------------------------- Laser Config ----------------------------------#
    ##################################################################################
    
    # laser_obj = None
    if(not mock_robot):
        laser_obj = Laser_Arduino()  # controls whether laser is on or off
        laser_on = False
        laser_obj.set_output(laser_on)

    
    ##################################################################################
    #----------------------------- Backend Connection -------------------------------#
    ##################################################################################

    def send_fn() -> str:
        current_state = robot_controller.get_current_pose()
        # TODO send other status (laser on, current path, etc)
        return RobotSchema.from_pose(current_state@np.linalg.inv(home_pose)).to_str()
    
    def recv_fn(msg: str):
        data = json.loads(msg)
        desired_state.update(data)
        if(desired_state.raster_mask is not None):
            # Do raster 
            image_bytes = desired_state.raster_mask.encode('utf-8')
            numpy_array = np.frombuffer(image_bytes, np.uint8)
            cv2.imdecode(numpy_array, cv2.IMREAD_COLOR)
            print("recieved raster png")
        elif(desired_state.path is not None and len(desired_state.path) > 1):
            # Do path
            target = desired_state.path.pop(0)
            desired_state.z = 0
            desired_state.update(data=target)

        # TODO keep looping
        desired_state.go_to_pose(home_t=home_pose, robot_controller=robot_controller)
        
        # TODO enable/disable laser
        # laser_obj.set_output(desired_pose.isLaserOn)

    backend_connection = BackendConnection(
        send_fn=send_fn,
        recv_fn=recv_fn,
        mocking=mock_robot
    )
    asyncio.create_task(backend_connection.connect_to_websocket())
    
    ##################################################################################
    #----------------------------- Camera Calibration -------------------------------#
    ##################################################################################
    
    if camera_calibration:
        pass

    # await asyncio.Future()
    
    print("Cameras are ready")
    therm_cam.start_stream() # Start camera stream
    # rgbd_cam.start_stream()
    print("Starting Streams ")
    b = Broadcast(mock_robot)
    print(f"Broadcast connection status: {b.connect()}")
    
    while (True):
        # rgbd_cam.display_all_streams() #Test Streaming
        latest = therm_cam.get_latest()
        await asyncio.sleep(0.01)
        img = None
        if latest is not None:
            if('themal') in latest.keys():
                img = latest['thermal']
                max = np.max(img)
                img = img / max
                img = img * 254
            elif('image') in latest.keys():
                img = latest['image']
            else:
                continue
        if(b.connected):
            b.publish_frame(img)
        else:
            b.connect()
            print('connecting...')
            time.sleep(2)
            
        
    
    therm_cam.deinitialize_cam()

    
    

if __name__ == "__main__":
    asyncio.run(main())