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
        self.show_path = True
        
        self._last_pose_ui = 0.0

        self.virtual_fixture, self.dx, self.dy, self.distance_field = self.generate_virtual_fixture()
        self.vf_valid_flag = None
        
        
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
        
    def generate_virtual_fixture(self, img = np.zeros((720, 1280), dtype=np.uint8)):
        '''
        Returns:
        Virtual fixture mask (not allowed @ true)
        dx
        dy
        distance_field
        '''
        # Return mask, gradient field
        # TODO keep virtual fixture in robot schema, not handlers
        virtual_fixture = img

        # cv2.rectangle(virtual_fixture, (0,0), (400,200), color=1, thickness=-1)
        # cv2.ellipse(virtual_fixture, center=(900,250), axes=(160,150), color=1, thickness=-1, angle=0, startAngle=0, endAngle=180)
        
        inverted_virtual_fixture = ~(virtual_fixture.astype(bool))
        distance_field = cv2.distanceTransform(inverted_virtual_fixture.astype(np.uint8), cv2.DIST_L2, 5)
        distance_field = cv2.GaussianBlur(distance_field, (0,0), 10)
        dx, dy = cv2.Sobel(distance_field, cv2.CV_32F, 1, 0, ksize=3), cv2.Sobel(distance_field, cv2.CV_32F, 0, 1, ksize=3)
        
        return inverted_virtual_fixture, dx, dy, distance_field

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
        
    def _read_mask(self, mask):
        data_str = mask
        image_bytes = base64.b64decode(data_str)
        numpy_array = np.frombuffer(image_bytes, np.uint8)
        img = cv2.imdecode(numpy_array, cv2.IMREAD_UNCHANGED)
        img = cv2.resize(img, (1280, 720))
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        return gray       
            
    def _read_raster(self):
        img = self._read_mask(self.desired_state.raster_mask)
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
        fig.savefig("raster path.png")
        return path
    
    def _read_fixtures(self):
        gray = self._read_mask(self.desired_state.fixtures_mask)
        self.virtual_fixture, self.dx, self.dy, self.distance_field = self.generate_virtual_fixture(img=gray)
        # cv2.imwrite("Virtural Fixtures.png", (self.virtual_fixture.astype(np.uint8)) * 255)
    
    def _track_virtual_fixtures(self, pixel):
        x, y = pixel
        if(x is not None and y is not None):
            self.vf_valid_flag = self.virtual_fixture[y, x] # 1 is valid
            # print(f"[Virtual Fixtures]: {'Valid' if self.vf_valid_flag else 'Not Valid'} Position")
            laser_on = self.desired_state.isLaserOn and self.vf_valid_flag
            
            self.laser_obj.vf_valid_flag = self.vf_valid_flag # should be handle on its own, double precaution
            self.laser_obj.set_output(laser_on)
            
    def _do_current_position(self):
        now = time.time()
        if now - self._last_pose_ui < 1/75.0:  # 75 Hz
            return
        self._last_pose_ui = now
    
        warped = self.desired_state.isTransformedViewOn
        curr_position = self.robot_controller.current_robot_to_world_position()
        current_pixel_location = self.cam_reg.get_world_m_to_UI(self.cam_type, curr_position, warped)[0].astype(np.int16)
        
        self._track_virtual_fixtures(current_pixel_location)
        self.desired_state.laserX, self.desired_state.laserY = current_pixel_location
    
    def _do_current_thermal_info(self):
        if self.desired_state.heat_markers != None:
            if(len(self.desired_state.heat_markers) == 0 ):
                return
            temps = np.zeros(len(self.desired_state.heat_markers))
            markers = np.array([[pixel['x'], pixel['y']] for pixel in self.desired_state.heat_markers])
            # print("[Temperature Markers] Marker Locations: ", markers)
            warped_view = self.desired_state.isTransformedViewOn
            pixel_loc = self.cam_reg.get_UI_to_thermal(markers, warped_view)
            therm_img = self.cam_reg.get_cam_latest('thermal')
            # print("[Temperature Markers] Pixel locations: ", pixel_loc)

            xs = pixel_loc[:, 0]
            ys = pixel_loc[:, 1]

            h, w = therm_img.shape[:2]

            valid = (xs >= 0) & (xs < w) & (ys >= 0) & (ys < h)

            temps = np.full(len(pixel_loc), np.nan, dtype=np.float32)
            temps[valid] = therm_img[ys[valid], xs[valid]]
            
            for i, marker in enumerate(self.desired_state.heat_markers):
                if valid[i]:
                    self.desired_state.heat_markers[i]["temp"] = float(therm_img[ys[i], xs[i]])
                else:
                    marker["temp"] = None
                    
    def get_heat_overlay(self, img):
        mask = self._read_mask(self.desired_state.heat_mask)
        heat_img, selection, min_temp, max_temp = self.cam_reg.heat_overlay(img, mask, invert=True)
        self.desired_state.averageHeat = max_temp #TODO find average heat of current robot kernal pixel
        return heat_img
            
    
    def _read_path(self):
        # TODO determine cam type from desired state
        # TODO convert path from List
        path = np.array([[d['x'], d['y']] for d in self.desired_state.path])
        return path
    
    def _do_create_path(self, path):
        pixels = path
        path = None
        
        warped_view = self.desired_state.isTransformedViewOn
        robot_path = self.cam_reg.get_UI_to_world_m(
                self.cam_type, 
                pixels, 
                warped_view, 
                z = self.working_height)
            
        speed = self.desired_state.speed if self.desired_state.speed != None else 0.01 # m/s
        traj = self.robot_controller.create_custom_trajectory(robot_path, speed)
        return traj
    
    def _do_show_path(self, traj):
        target_positions = traj.get_path_position()
        warped = self.desired_state.isTransformedViewOn
        pixels = self.cam_reg.get_world_m_to_UI(self.cam_type, target_positions, warped)
        pixels = np.asarray(pixels, dtype=np.int16)
        self.cam_reg.get_path(pixels)
        self.cam_reg.display_path = True
    
    
    def _do_path(self, traj):
        # TODO determine cam type from desired state
        # TODO convert path from List
        if self.show_path:
            self._do_show_path(traj)
        
        # TODO uncomment controller 
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
            
        # self.laser_obj.set_output(False)

    def _do_live_control(self):
        # If no new message in 200ms, stop
        if(self._input_downtime() > .12):
            self._do_hold_pose()
        else:
            pixel = np.array([[self.desired_state.x, self.desired_state.y]])
            
            warped_view = self.desired_state.isTransformedViewOn
            target_world_point = self.cam_reg.get_UI_to_world_m(
                self.cam_type, 
                pixel, 
                warped_view, 
                z = self.working_height)[0]
    
            target_pose = np.eye(4)
            target_pose[:3, -1] = target_world_point
            target_vel = self.robot_controller.live_control(target_pose, 0.05)
            # TODO Multiply velocity controller in unit component direction * max(min_speed, min(1, (distance / max_distance)))
            
            
            self.robot_controller.set_velocity(target_vel, np.zeros(3))

    

    async def main_loop(self):
        # Yield to other threads (video stream, websocket comms)
        await asyncio.sleep(0.0001)
        
        if(self.desired_state.isRobotOn != self.prev_robot_on):
            self._do_hold_pose() 
        
        self._do_current_thermal_info()
        # print(self.desired_state.heat_markers)
        self._do_current_position()

        if(self.desired_state.isRobotOn):          
            
            # print("loop",
            # "raster?", self.desired_state.raster_mask is not None,
            # "path?", self.desired_state.path is not None,
            # "Path event?", self.desired_state.pathEvent,
            # "traj_running?", self.robot_controller.is_trajectory_running())
            
            
            if(self.desired_state.fixtures_mask is not None):
                self._read_fixtures()
                self.desired_state.fixtures_mask = None
                
            
            # TODO, always recieving raster_mask 
            
            if(self.desired_state.raster_mask is not None
               and not self.robot_controller.is_trajectory_running()):
                self.desired_state.x = None
                self.desired_state.y = None
                print("Raster Trigger")
                raster = self._read_raster()
                if len(raster) < 1:
                    print("[Warning] : Raster path is empty")
                else: 
                    traj = self._do_create_path(raster)
                    self._do_path(traj)
                    
                self.desired_state.raster_mask = None
                self.desired_state.path = None
                
            elif(self.desired_state.path is not None and len(self.desired_state.path) > 1
                 and not self.robot_controller.is_trajectory_running()):
                print("Path Trigger")
                path = self._read_path()
                traj = self._do_create_path(path)
                self._do_path(traj)
                
                self.desired_state.x = None
                self.desired_state.y = None
                self.desired_state.path = None
                
            elif(self.desired_state.x is not None and self.desired_state.y is not None 
                 and not self.robot_controller.is_trajectory_running()):
                if (self.desired_state.x >= 0 and self.desired_state.y >= 0):
                    # TODO check outside boundary
                    # Disable laser
                    # Pull laser back into closest valid position
                    # print(f"Live controller trigger {self.desired_state.x}, {self.desired_state.y}")
                    self._do_live_control()
                    self.desired_state.path = None
                    
                    
                    
        else:
            self._do_hold_pose()
                
        self.prev_robot_on = self.desired_state.isRobotOn
        