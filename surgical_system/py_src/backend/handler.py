from robot.robot import RobotSchema
from robot.mock_robot_controller import MockRobotController
from registration.mock_camera_registration import MockCameraRegistration
from laser_control.mock_laser import MockLaser
import numpy as np
from backend.listener import BackendConnection
import time
import json
import cv2
from dataclasses import asdict
import asyncio
import base64
from motion_planning.motion_planning import Motion_Planner
import matplotlib.pyplot as plt


class Handler:
    def __init__(self, desired_state: RobotSchema, robot_controller: MockRobotController, cam_reg: MockCameraRegistration, laser_obj: MockLaser, start_pose, mock_robot):
        self.desired_state = desired_state
        self.robot_controller = robot_controller
        self.last_update_time = time.time()
        self.home_tf = robot_controller.load_home_pose()
        self.start_pose = start_pose
        self.cam_reg = cam_reg
        self.laser_obj = laser_obj
        self.cam_type = "color"
        self.prev_robot_on = False
        self.working_height = 0.0

        initial_pose, _ = robot_controller.get_current_state()
        desired_state.update(asdict(RobotSchema.from_pose(initial_pose@np.linalg.inv(self.home_tf))))
        desired_state.isLaserOn = False
        desired_state.isRobotOn = False

        backend_connection = BackendConnection(
            send_fn=self._send_fn,
            recv_fn=self._recv_fn,
            mocking=mock_robot
        )
        asyncio.create_task(backend_connection.connect_to_websocket())

    def _input_downtime(self):  
        return time.time() - self.last_update_time
    
    def _send_fn(self) -> str:
        current_pose, _ = self.robot_controller.get_current_state()
        status = RobotSchema.from_pose(np.linalg.inv(self.home_tf)@current_pose)
        status.isLaserOn = self.desired_state.isLaserOn # TODO Separate variable for on & enabled? Need read-only portions of schema?
        status.isRobotOn = self.desired_state.isRobotOn # TODO get from ???
        return status.to_str()
    
    def _recv_fn(self, msg: str):
        self.last_update_time = time.time()
        data = json.loads(msg)
        self.desired_state.update(data)
        if(self.desired_state.isThermalViewOn):
            self.cam_type = "thermal"
        else:
            self.cam_type = "color"
            
    def _read_raster(self):
        data_str = self.desired_state.raster_mask
        image_bytes = base64.b64decode(data_str)
        numpy_array = np.frombuffer(image_bytes, np.uint8)
        img = cv2.imdecode(numpy_array, cv2.IMREAD_UNCHANGED)
        img = cv2.resize(img, (1280, 720)) # TODO change this to reflect correct transformation
        img = Motion_Planner.fill_in_shape(img)
        path = Motion_Planner.raster_pattern(img, pitch = 8)
        print("Raster Path: ", path)
        fig, ax = plt.subplots(figsize=(8,4))
        # ax.imshow(img, cmap='gray')
        if len(path) > 1:
            xs = [p[0] for p in path]
            ys_plot = [p[1] for p in path]
            ax.plot(xs, ys_plot, linewidth=1)  # default color
        ax.set_axis_off()
        fig.savefig("test.png")
        return path
        
    '''
    path = []
    if(desired_state.path is not None and len(desired_state.path) > 1):
        # Do path
        if len(path) == 0:
            print("Tracing path")
            path = desired_state.path 
            desired_state.path = None
            path = np.array([[d["x"], d["y"]] for d in path], dtype=float)
        else:
            # Raster pattern branch
            path = np.array(path, dtype=float)
            

            # If Motion_Planner returned (row, col) = (y, x),
            # convert to (x, y) to match camera_reg expectations:
            #   path[:,0] = row (y), path[:,1] = col (x)
            #   pixels[:,0] = x = col
            #   pixels[:,1] = y = row
            h, w = img.shape[:2]
            pixels = np.zeros_like(path)
            pixels[:, 0] = path[:, 1]         # x = col
            pixels[:, 1] = path[:, 0]  
            
        
    '''    
    
    def _do_current_position(self):
        curr_position = self.robot_controller.current_robot_to_world_position()
        
        if self.desired_state.isTransformedViewOn:
            pixel = self.cam_reg.real_to_world(curr_position, self.cam_type)[0]
            self.desired_state.laserX, self.desired_state.laserY = pixel
        # else:
        #     target_world_point = self.cam_reg.pixel_to_world(pixel, cam_type=self.cam_type, z=self.working_height)[0]
    
    def _do_current_thermal_info(self):
        pass
    
    def _read_path(self):
        # TODO determine cam type from desired state
        # TODO convert path from List
        path = np.array([[d['x'], d['y']] for d in self.desired_state.path])
        return path

    def _do_path(self, path):
        # TODO determine cam type from desired state
        # TODO convert path from List
        pixels = path
        path = None
        # pixels = self.cam_reg.moving_average_smooth(pixels, window=5)
        if self.desired_state.isTransformedViewOn:
            robot_path = self.cam_reg.world_to_real(pixels, cam_type=self.cam_type, z = self.working_height)
        else:
            robot_path = self.cam_reg.pixel_to_world(pixels, cam_type=self.cam_type, z = self.working_height)
        speed = self.desired_state.speed if self.desired_state.speed != None else 0.01 # m/s
        traj = self.robot_controller.create_custom_trajectory(robot_path, speed)
        # self.laser_obj.set_output(self.desired_state.isLaserOn)
        self.robot_controller.run_trajectory(traj, blocking=False, laser_on=self.desired_state.isLaserOn)
        
    def _do_hold_pose(self):
        current_pose, current_vel = self.robot_controller.get_current_state()
        # Stop robot, no drift
        if np.linalg.norm(current_vel[:3]) > 2e-5:
            self.robot_controller.set_velocity(np.zeros(3), np.zeros(3))
        else:
            self.robot_controller.go_to_pose(current_pose, blocking=False)
        self.desired_state.x = None
        self.desired_state.y = None
            
        self.laser_obj.set_output(False)

    def _do_live_control(self):
        # If no new message in 200ms, stop
        if(self._input_downtime() > .12):
            self._do_hold_pose()
        else:
            pixel = np.array([[self.desired_state.x, self.desired_state.y]])
            if self.desired_state.isTransformedViewOn:
                target_world_point = self.cam_reg.world_to_real(pixel, cam_type=self.cam_type, z=self.working_height)[0]
            else:
                target_world_point = self.cam_reg.pixel_to_world(pixel, cam_type=self.cam_type, z=self.working_height)[0]
            target_pose = np.eye(4)
            # print(target_world_point)
            target_pose[:3, -1] = target_world_point
            target_vel = self.robot_controller.live_control(target_pose, 0.05)
            self.robot_controller.set_velocity(target_vel, np.zeros(3))

            self.laser_obj.set_output(self.desired_state.isLaserOn)
    

    async def main_loop(self):
        # Yield to other threads (video stream, websocket comms)
        await asyncio.sleep(0.0001)
        
        if(self.desired_state.isRobotOn != self.prev_robot_on):
            self._do_hold_pose() 
            
        # print(self.desired_state.heat_markers)
        self._do_current_position()

        if(self.desired_state.isRobotOn):
            # TODO interrupt trajectory?
            
            # print("loop",
            # "raster?", self.desired_state.raster_mask is not None,
            # "path?", self.desired_state.path is not None,
            # "traj_running?", self.robot_controller.is_trajectory_running())
            
            
            if(self.desired_state.raster_mask is not None
               and not self.robot_controller.is_trajectory_running()):
                self.desired_state.x = None
                self.desired_state.y = None
                print("Raster Trigger")
                raster = self._read_raster()
                if len(raster) < 1:
                    print("[Warning] : Raster path is empty")
                else: 
                    self._do_path(raster)
                self.desired_state.raster_mask = None
                self.desired_state.path = None
                
            elif(self.desired_state.path is not None and len(self.desired_state.path) > 1
                 and not self.robot_controller.is_trajectory_running()):
                print("Path Trigger")
                self.desired_state.x = None
                self.desired_state.y = None
                self.desired_state.path = None
                pass
            #     path = self._read_path()
            #     self._do_path(path)
            #     self.desired_state.path = None

            elif(self.desired_state.x is not None and self.desired_state.y is not None 
                 and not self.robot_controller.is_trajectory_running()):
                if (self.desired_state.x >= 0 and self.desired_state.y >= 0):
                    print(f"Live controller trigger {self.desired_state.x}, {self.desired_state.y}")
                    self._do_live_control()
                    self.desired_state.path = None
                    
                    
                    
        else:
            self._do_hold_pose()
                
        self.prev_robot_on = self.desired_state.isRobotOn
        