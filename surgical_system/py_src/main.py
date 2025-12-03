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
    cam_type = "color"
    
    if(not mock_robot):
        therm_cam = ThermalCam(IRFormat="TemperatureLinear10mK", height=int(480/window_scale),frame_rate="Rate50Hz",focal_distance=0.2)
        rgbd_cam = RGBD_Cam() #Runs a thread internally
        rgbd_cam.set_default_setting() # Auto-exposure
    else:
        therm_cam = MockCamera(cam_type=cam_type)
        rgbd_cam = MockCamera(cam_type=cam_type)
        

    
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

    def send_fn() -> str:
        current_pose, _ = robot_controller.get_current_state()
        status = RobotSchema.from_pose(current_pose@np.linalg.inv(home_pose))
        status.isLaserOn = desired_state.isLaserOn
        return status.to_str()
    
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
            robot_waypoints = camera_reg.pixel_to_world(desired_state.path.values(), cam_type=cam_type)
            print(robot_waypoints[0])
            # traj = robot_controller.create_trajectory(gains)

        # TODO keep looping
        # desired_state.go_to_pose(home_t=home_pose, robot_controller=robot_controller)
        recv_fn.last_update = time.time()       
        
    recv_fn.last_update = None

    # TODO this is a hotfix, overload update to accept a robot schema
    initial_pose, _ = robot_controller.get_current_state()
    desired_state.update(asdict(RobotSchema.from_pose(initial_pose@np.linalg.inv(home_pose))))

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

    
    start_pose[2,3] = 0.0
    robot_controller.go_to_pose(start_pose@home_pose,1) # Send robot to start position
    
    while (True):
        # Backend pose update
        await asyncio.sleep(0.0001)
        
        # Robot velocity controller        
        current_time = time.time()
        diff = 1
        if(recv_fn.last_update is not None):
            diff = current_time - recv_fn.last_update
        # If no new message in 200ms, stop
        if(diff > .12):
            current_pose, current_vel = robot_controller.get_current_state()
            # Stop robot, no drift
            if np.linalg.norm(current_vel[:3]) > 2e-5:
                robot_controller.set_velocity(np.zeros(3), np.zeros(3))
            else:
                robot_controller.go_to_pose(current_pose, blocking=False)
                
            laser_obj.set_output(False)
        else:
            target_world_point = camera_reg.pixel_to_world(np.array([desired_state.x, desired_state.y]), cam_type=cam_type, z=start_pose[2,3])
            target_pose = np.eye(4)
            target_pose[:3, -1] = target_world_point
            target_vel = robot_controller.live_control(target_pose, 0.05) # TODO given current and desired pose, set vel
            robot_controller.set_velocity(target_vel, np.zeros(3))
            
            laser_obj.set_output(desired_state.isLaserOn)
            
            
        # Camera frame publishing
        latest = camera_reg.get_cam_latest(cam_type=cam_type)
        if isinstance(latest, dict):
            latest = latest.get(cam_type, None)
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