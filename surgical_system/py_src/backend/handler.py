from robot.robot import RobotSchema
from robot.mock_robot_controller import MockRobotController
from registration.mock_camera_registration import MockCameraRegistration
from laser_control.mock_laser import MockLaser
from motion_planning.motion_planning import Motion_Planner
from backend.listener import BackendConnection
from robot.controllers.trajectory_controller import TrajectoryController
from dataclasses import asdict
from typing import Dict, Any, Tuple

import numpy as np
import matplotlib.pyplot as plt

import time, math, json, cv2, asyncio, base64


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
        self.desired_state.current_height = None
        self.show_path = True
        self.current_traj = None
        self._last_pose_ui = 0.0
        self.mock_robot = mock_robot

        self.path_display_pixels = None
        self.new_path_flag = False
        
        # Recording Data
        self._start_recording_time = 0
        self._current_recording_time = 0; 
        self.recording_data = False # TODO TEMP var
        
        
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
        
    def generate_virtual_fixture(self, img = np.ones((720, 1280), dtype=np.uint8)):
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
        
        return virtual_fixture.astype(bool), dx, dy, distance_field

    def _input_downtime(self):  
        return time.time() - self.last_update_time
    
    def _send_fn(self) -> str:
        current_pose, _ = self.robot_controller.get_current_state()
        status = RobotSchema.from_pose(np.linalg.inv(self.home_tf)@current_pose)
        status.isLaserOn = self.desired_state.isLaserOn # TODO Separate variable for on & enabled? Need read-only portions of schema?
        status.isRobotOn = self.desired_state.isRobotOn # TODO get from ???
        status.heat_markers = self.desired_state.heat_markers
        # print(self.desired_state.heat_markers)
        status.maxHeat = self.desired_state.maxHeat
        if not self.mock_robot:
            status.laserX, status.laserY = int(self.desired_state.laserX), int(self.desired_state.laserY)
            status.current_height =  self.desired_state.current_height 
        else:
            status.laserX, status.laserY = status.x, status.y
            status.current_height =  np.pi
            
        if self.new_path_flag:
            status.path_preview = self.desired_state.path_preview
            print(f"[send_fn Handler] Sending Speed: {self.desired_state.path_preview['time']}")
            self.new_path_flag = False
            
        return status.to_str()
    
    def _recv_fn(self, msg: str):
        self.last_update_time = time.time()
        data = json.loads(msg)
        self.desired_state.update(data)
        
        
###------------------------ Mask Reader -------------------####    


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
        spacing = self.desired_state.density
        if spacing is None or math.isnan(spacing):
            return []
        spacing = int(spacing) # TODO calculate lines per distance 
        
        # print(f"[Spacing (pixels)]: {spacing}")
        # path = Motion_Planner.raster_pattern(img, pitch = spacing) # pixel spacing
        #TODO check if raster is valid 

    
    
        polygon, edge = Motion_Planner._create_polygon(img)
        
        if polygon is None:
            print("No shape found ")
            return []
        
        path = Motion_Planner.poly_raster(
            polygon,
            spacing=spacing,        # pixels
            theta_deg=45.0,      # angle
            margin=5.0          # inward offset
        )
        
        # print("Raster Path: ", path)
        fig, ax = plt.subplots(figsize=(8,4))
        # ax.imshow(img, cmap='gray')
        if len(path) > 1:
            xs = [p[0] for p in path]
            ys_plot = [p[1] for p in path]
            ax.plot(xs, ys_plot, linewidth=1)  # default color
        ax.set_axis_off()
        fig.savefig("raster path.png")
        return path
    
###------------------------ Virtual Fixtures -------------------####    

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
            
###------------------------ Robot Tracking -------------------####                
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
    
    
###------------------------ THERMAL OVERLAY -------------------####    

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
        heat_img, selection, therm_img, max_temp = self.cam_reg.heat_overlay(img, mask, invert=True)
        if selection is not None and selection.size != 0:
            self.desired_state.maxHeat = float(max_temp) if max_temp is not None else float(np.max(therm_img)) #TODO find average heat of current robot kernal pixel
        # print(f"Max temp: {max_temp}")
        # self.desired_state.heat_mask = None
        return heat_img
            
###------------------------ Planned Path Control -------------------####    


    def _read_path(self):
        path = np.array([[d['x'], d['y']] for d in self.desired_state.path])
        return path
    
    def _do_create_path(self, path):
        pixels = path
        path = None
        pixels = Motion_Planner.rdp(pixels, epsilon=1)
        pixels = Motion_Planner.smooth_corners_fillet(pixels, radius=3, n_arc=20)
        
        fig, ax = plt.subplots(figsize=(8,4))
        # ax.imshow(img, cmap='gray')
        if len(pixels) > 1:
            xs = [p[0] for p in pixels]
            ys_plot = [p[1] for p in pixels]
            ax.plot(xs, ys_plot, linewidth=1)  # default color
        ax.set_axis_off()
        fig.savefig("pixels path.png")
        plt.close()
        
        warped_view = self.desired_state.isTransformedViewOn
        # print("Warped Path: ", warped_view)
        robot_path = self.cam_reg.get_UI_to_world_m(
                self.cam_type, 
                pixels, 
                warped_view, 
                z = self.working_height)
        # fig, ax = plt.subplots(figsize=(8,4))
        
        # if len(robot_path) > 1:
        #     xs = [p[0] for p in robot_path]
        #     ys_plot = [p[1] for p in robot_path]
        #     ax.plot(xs, ys_plot, linewidth=1)  # default color
        # ax.set_axis_off()
        # fig.savefig("World Positions after warp path.png")
            
        speed = self.desired_state.speed / 1000.0 if self.desired_state.speed != None else 0.01 # m/s
        traj = self.robot_controller.create_custom_trajectory(robot_path, speed)
        target_pixels, total_time = self._do_show_path(traj)
        self._package_path(target_pixels, total_time)
        return traj
    
    def _do_show_path(self, traj: TrajectoryController):
        target_positions = traj.get_path_position()
        fig, ax = plt.subplots(figsize=(8,4))
        # ax.imshow(img, cmap='gray')
        # if len(target_positions) > 1:
        #     xs = [p[0] for p in target_positions]
        #     ys_plot = [p[1] for p in target_positions]
        #     ax.plot(xs, ys_plot, linewidth=1)  # default color
        # ax.set_axis_off()
        # fig.savefig("target_positions path.png")
        
        warped = self.desired_state.isTransformedViewOn
        pixels = self.cam_reg.get_world_m_to_UI(self.cam_type, target_positions, warped)
        pixels = np.asarray(pixels, dtype=np.int16)
        # if len(pixels) > 1:
        #     xs = [p[0] for p in pixels]
        #     ys_plot = [p[1] for p in pixels]
        #     ax.plot(xs, ys_plot, linewidth=1)  # default color
        # ax.set_axis_off()
        # fig.savefig("pixels path.png")
        self.path_display_pixels = pixels
        return pixels, traj.total_path_time
    
    def _package_path(self, pixels, total_time):
        x, y = list(pixels[:, 0].astype(np.float64)), list(pixels[:, 1].astype(np.float64))
        # print(pixels)
        time = [total_time]
        # print(f"Total Time: {total_time}")
        # print("[package_path Handler] pixels:", pixels)
        self.desired_state.path_preview = {
            "x" : x,
            "y" : y,
            "time": time
        }
        self.new_path_flag = True
     
    def _do_path(self, traj: TrajectoryController):
        self.robot_controller.run_trajectory(traj, blocking=False, laser_on=self.desired_state.isLaserOn)
        
###------------------------ Live Control ------------------####
        
    def _do_hold_pose(self):
        current_pose, current_vel = self.robot_controller.get_current_state()
        # Stop robot, no drift
        # print(np.linalg.norm(current_vel[:3]))
        if np.linalg.norm(current_vel[:3]) > 2e-5:
            # print("Setting speed 0")
            # self.robot_controller.set_velocity(np.zeros(3), np.zeros(3))
            target_world_point = self.robot_controller.current_robot_to_world_position()
            target_world_point[-1] = self.working_height
                
            target_pose = np.eye(4)
            target_pose[:3, -1] = target_world_point
            target_vel = self.robot_controller.live_control(target_pose, 0.05, KP = 0.5)
            self.robot_controller.set_velocity(target_vel, np.zeros(3))
        else:
            # print("holding")
            self.robot_controller.go_to_pose(current_pose, blocking=False)
        self.desired_state.x = None
        self.desired_state.y = None
            
        # self.laser_obj.set_output(False)

    def _do_live_control(self, height_diff):
        # If no new message in 120 ms, stop
        # print(height_change)
        # print(f"[INPUT DOWN TIME] {self._input_downtime()}")
        # print(f"[Height Diff] {np.abs(height_diff)}")
        if(self._input_downtime() > .12 and np.abs(height_diff) < 0.001): # 1mm
            # print("[Live Control] Holding Position")
            self._do_hold_pose()
        else:
            if self.desired_state.x is not None and  self.desired_state.y is not None:
                if (self.desired_state.x < 0 and self.desired_state.y < 0):
                    return
                pixel = np.array([[self.desired_state.x, self.desired_state.y]])
                
                warped_view = self.desired_state.isTransformedViewOn
                target_world_point = self.cam_reg.get_UI_to_world_m(
                    self.cam_type, 
                    pixel, 
                    warped_view, 
                    z = self.working_height)[0]
                self.laser_obj.set_output(self.desired_state.isLaserOn)
            else:
                target_world_point = self.robot_controller.current_robot_to_world_position()
                target_world_point[-1] = self.working_height
                
            target_pose = np.eye(4)
            target_pose[:3, -1] = target_world_point
            live_control_speed = self.desired_state.speed / 1000.0 if self.desired_state.speed != None else 0.01
            print(f"[Handler Live Control] Speed: {self.desired_state.speed}")
            target_vel = self.robot_controller.live_control(target_pose, live_control_speed, KP = 5.0)
            # TODO Multiply velocity controller in unit component direction * max(min_speed, min(1, (distance / max_distance)))
            
            
            self.robot_controller.set_velocity(target_vel, np.zeros(3))

###------------------------ Data Collection ------------------####

    def _store_camera_feed(self, 
                           rgb_img: np.ndarray = np.array([]), 
                           therm_img: np.ndarray = np.array([])):
        pass
    
    def _store_robot_state(self,  
                           q: np.ndarray, # (7,)
                           dq: np.ndarray): # (7,))
        pass
    
    def _store_user_input(self,
                          mode: str,                     # "draw", "live_control"
                          payload: Dict[str, Any]):       # markers, path, etc.)
        pass 

    async def main_loop(self):
        # Yield to other threads (video stream, websocket comms)
        await asyncio.sleep(0.0001)
        
        if(self.desired_state.isRobotOn != self.prev_robot_on):
            self._do_hold_pose() 
        
        self._do_current_thermal_info()
        # print(self.desired_state.heat_markers)
        self._do_current_position()
        
        
        if(self.desired_state.raster_mask is not None
               and not self.robot_controller.is_trajectory_running()):
                self.desired_state.x = None
                self.desired_state.y = None
                self.laser_obj.set_output(False)
                print("Raster Trigger")
                raster = self._read_raster()
                if len(raster) < 1:
                    print("[Warning] : Raster path is empty")
                    self._package_path(np.empty((1, 2), dtype=float), -1)
                else: 
                    self.current_traj = self._do_create_path(raster)
                    
                self.desired_state.raster_mask = None
                self.desired_state.path = None
                
        elif(self.desired_state.path is not None and len(self.desired_state.path) > 1
                and not self.robot_controller.is_trajectory_running()):
            print("Path Trigger")
            self.laser_obj.set_output(False)
            path = self._read_path()
            
            self.current_traj = self._do_create_path(path)
            
            self.desired_state.x = None
            self.desired_state.y = None
            self.desired_state.path = None
            
        self.desired_state.current_height = self.robot_controller.current_robot_to_world_position()[-1]

        if(self.desired_state.isRobotOn):          
            self.working_height = self.desired_state.height / 100.0 if self.desired_state.height else  0 # cm to m
            # print(f"[Robot Height] {self.working_height} m")
                
            # print("loop",
            # "raster?", self.desired_state.raster_mask is not None,
            # "path?", self.desired_state.path is not None,
            # "Path event?", self.desired_state.pathEvent,
            # "traj_running?", self.robot_controller.is_trajectory_running())
            height_diff = self.working_height - self.robot_controller.current_robot_to_world_position()[-1]
            height_change = np.abs(height_diff) > 0.0001 #0.1 mm
            # print(f"[Robot Height] Height Change: {height_change}")
            if(self.robot_controller.is_trajectory_running() and self.path_display_pixels is not None):
                self.cam_reg.display_path = True
                self.cam_reg.get_path(self.path_display_pixels)
            else:
                self.cam_reg.display_path = False
                # self.path_display_pixels = None
            
            if(self.desired_state.fixtures_mask is not None):
                self._read_fixtures()
                self.desired_state.fixtures_mask = None
                
            if(self.current_traj is not None and self.desired_state.executeCommand
                   and not self.robot_controller.is_trajectory_running()):
                print("Executing Path")
                self._do_path(self.current_traj)
                self.desired_state.executeCommand = None
                self.current_traj = None
                
            elif(((self.desired_state.x is not None and self.desired_state.y is not None) or height_change)
                   and not self.robot_controller.is_trajectory_running()):
                
                    # print(f"Live controller trigger {self.desired_state.x}, {self.desired_state.y}")
                self._do_live_control(height_diff)
                self.desired_state.path = None
            # elif(not self.robot_controller.is_trajectory_running())
            
                
                    
                    
        else:
            self._do_hold_pose()
            self.laser_obj.set_output(False)
                
        self.prev_robot_on = self.desired_state.isRobotOn
        