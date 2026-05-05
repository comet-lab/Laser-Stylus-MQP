from robot.robot import RobotSchema
from robot.mock_robot_controller import MockRobotController
from registration.mock_camera_registration import MockCameraRegistration
from laser_control.mock_laser import MockLaser
from motion_planning.motion_planning import Motion_Planner
from backend.listener import BackendConnection
from robot.controllers.trajectory_controller import TrajectoryController
from robot.robot_fixtures import RobotFixtures, GridBoundary
from backend.datastorage import SystemDataStore, CameraFrame, RobotState, UserCommand
from dataclasses import asdict
from typing import Dict, Any, Tuple
from scipy.spatial.transform import Rotation

import numpy as np
import matplotlib.pyplot as plt

import time, math, json, cv2, asyncio, base64, os

from registration.transformations.depth_estimation import DepthEstimation


class Handler:
    def __init__(self, desired_state: RobotSchema, 
                 robot_controller: MockRobotController, 
                 cam_reg: MockCameraRegistration, 
                 laser_obj: MockLaser, 
                 start_pose, 
                 mock_robot,
                 data_storage: SystemDataStore):
        self.pathToCWD = os.getcwd()
        self.directory = self.pathToCWD 
        os.makedirs(os.path.join(self.pathToCWD, "plots"), exist_ok=True)
        
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
        self.recording_data = False 
        self.data_storage = data_storage
        self.desired_state.isRecordingOn = None
        
        
        boundary = self.cam_reg.meta_base_homography_data["boundary"] 
        if boundary is not None:
            print("[Handler] Loading Robot Fixtures")
        else:
            # Default 
            print("[Handler] Defaulting Robot Fixtures")
            boundary = [[-0.0215, -0.0215],
                        [ 0.0215, -0.0215],
                        [ 0.0215,  0.0215],
                        [-0.0215,  0.0215]] 
            
        self.robot_fixtures = RobotFixtures(boundary, include_boundary=False)
        self.robot_fixtures.plot_valid_region() # Debug
        
        
        self.virtual_fixture, self.dx, self.dy, self.distance_field = self.generate_virtual_fixture()
        self.vf_valid_flag = None
        
        self.hold_pose_flag = False
        self.hold_position = None
        self.hold_pose = None
        
        
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
        spacing = int(spacing) # TODO 
        polygon, edge = Motion_Planner._create_polygon(img)
        
        if polygon is None:
            print("No shape found ")
            return []
        
        edge_path = self._do_filter_prim_path(edge)
        
        path = Motion_Planner.poly_raster(
            polygon,
            spacing=spacing,        # pixels
            theta_deg=45.0,      # angle
            margin=1.0          # inward offset
        )
        
        full_path = path.copy()
        def unit(v, eps=1e-8):
                n = np.linalg.norm(v)
                return v / n if n > eps else np.zeros_like(v)
            
        # Connect raster with outer path 
        if len(path) > 0 and len(edge_path) > 0:
            # raster terminal direction
            path_dir = unit(path[-1] - path[-2])
            end_pt = path[-1]

            vecs = edge_path - end_pt
            dists = np.linalg.norm(vecs, axis=1)

            # 1) find closest edge point first
            closest_idx = np.argmin(dists)

            # 2) only consider a local neighborhood around that point
            window = 10   # tune this
            n = len(edge_path)
            candidate_idxs = [((closest_idx + k) % n) for k in range(-window, window + 1)]

            # 3) among local candidates, pick the one best aligned with path direction
            best_idx = closest_idx
            best_score = -np.inf

            for idx in candidate_idxs:
                v = edge_path[idx] - end_pt
                v_hat = unit(v)

                # favor forward direction, but keep locality through the restricted window
                score = np.dot(v_hat, path_dir)

                if score > best_score:
                    best_score = score
                    best_idx = idx

            start_idx = best_idx      
            
            edge_path = np.vstack([
                edge_path[start_idx:],
                edge_path[:start_idx]
            ]) 
            
            # path end direction
            path_dir = unit(edge_path[0] - path[-1])

            edge_forward = unit(edge_path[1] - edge_path[0])
            edge_backward = unit(edge_path[-1] - edge_path[0])
            
            # compare alignment
            forward_score = np.dot(path_dir, edge_forward)
            backward_score = np.dot(path_dir, edge_backward)

            # if backward matches better, flip traversal direction
            if backward_score > forward_score:
                edge_path = np.vstack([
                    edge_path[:1],
                    edge_path[:0:-1]
                ])
                

            # close loop if needed
            if not np.allclose(edge_path[0], edge_path[-1]):
                edge_path = np.vstack([edge_path, edge_path[0]])

            # avoid duplicate point at transition
            if np.allclose(full_path[-1], edge_path[0]):
                full_path = np.vstack([full_path, edge_path[1:]])
            else:
                full_path = np.vstack([full_path, edge_path])
                
            # self.plot_connection_debug(path, edge_path, start_idx, [edge_forward, edge_backward], arrow_scale=0.001)
        
        # print("Raster Path: ", path)
        fig, ax = plt.subplots(figsize=(8,4), dpi=300)
        # ax.imshow(img, cmap='gray')
        if len(full_path) > 1:
            xs = [p[0] for p in full_path]
            ys_plot = [p[1] for p in full_path]
            ax.plot(xs, ys_plot, linewidth=1)  # default color
        ax.set_axis_off()
        fig.savefig("plots/debug/raster path.png")
        plt.close(fig)
        return full_path

    


    def plot_connection_debug(self, path, edge_path, start_idx, direction, arrow_scale=100.0, closed=True):
        """
        Visualize:
        - raster path
        - edge contour
        - chosen edge start point
        - raster terminal direction
        - edge forward/backward directions
        - connector vector

        Args:
            path:      (N,2)
            edge_path: (M,2)
            start_idx: chosen index on edge_path
            arrow_scale: multiplier to make unit vectors visible
            closed: whether edge_path is a closed contour
        """
        
        def unit(v, eps=1e-8):
            n = np.linalg.norm(v)
            return v / n if n > eps else np.zeros_like(v)
    
        path = np.asarray(path, dtype=float)
        edge_path = np.asarray(edge_path, dtype=float)

        if len(path) < 2:
            raise ValueError("path must have at least 2 points")
        if len(edge_path) < 3:
            raise ValueError("edge_path must have at least 3 points")

        end_pt = path[-1]
        start_pt = edge_path[start_idx]

        # raster terminal direction
        path_dir = unit(path[-1] - path[-2])

        if closed:
            prev_idx = (start_idx - 1) % len(edge_path)
            next_idx = (start_idx + 1) % len(edge_path)
        else:
            prev_idx = max(start_idx - 1, 0)
            next_idx = min(start_idx + 1, len(edge_path) - 1)
            
        edge_forward, edge_backward = direction

        connector_dir = unit(start_pt - end_pt)

        forward_score = np.dot(path_dir, edge_forward)
        backward_score = np.dot(path_dir, edge_backward)

        fig, ax = plt.subplots(figsize=(10, 8), dpi=400)

        # main paths
        ax.plot(edge_path[:, 0], edge_path[:, 1], label="Edge Path", linewidth=2)
        ax.plot(path[:, 0], path[:, 1], label="Raster Path", linewidth=2)

        # key points
        ax.scatter(path[0, 0], path[0, 1], s=120, marker='o', label="Raster Start", zorder=5)
        ax.scatter(end_pt[0], end_pt[1], s=140, marker='x', label="Raster End", zorder=6)
        ax.scatter(start_pt[0], start_pt[1], s=140, marker='s', label="Chosen Edge Point", zorder=1)

        # neighbor points on edge
        ax.scatter(edge_path[prev_idx, 0], edge_path[prev_idx, 1], s=70, marker='^', label="Edge Prev", zorder=5)
        ax.scatter(edge_path[next_idx, 0], edge_path[next_idx, 1], s=70, marker='v', label="Edge Next", zorder=5)

        # connector line
        ax.plot([end_pt[0], start_pt[0]], [end_pt[1], start_pt[1]], '--', linewidth=1.5, label="Connector")

        # arrows
        ax.quiver(
            end_pt[0], end_pt[1],
            path_dir[0] * arrow_scale, path_dir[1] * arrow_scale,
            angles='xy', scale_units='xy', scale=1,
            width=0.003, label="Path Dir"
        )

        ax.quiver(
            start_pt[0], start_pt[1],
            edge_forward[0] * arrow_scale, edge_forward[1] * arrow_scale,
            angles='xy', scale_units='xy', scale=1,
            width=0.003, label="Edge Forward"
        )

        ax.quiver(
            start_pt[0], start_pt[1],
            edge_backward[0] * arrow_scale, edge_backward[1] * arrow_scale,
            angles='xy', scale_units='xy', scale=1,
            width=0.003, label="Edge Backward"
        )

        ax.quiver(
            end_pt[0], end_pt[1],
            connector_dir[0] * arrow_scale, connector_dir[1] * arrow_scale,
            angles='xy', scale_units='xy', scale=1,
            width=0.002, label="Connector Dir"
        )

        ax.set_title(
            f"Connection Debug\n"
            f"forward_score={forward_score:.3f}, backward_score={backward_score:.3f}"
        )
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.axis("equal")
        ax.grid(True)
        ax.legend()
        plt.tight_layout()
        fig.savefig("plots/debug/raster path debug.png")
        plt.close(fig)
    
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
        curr_position = self.robot_controller.current_robot_to_world_position()
        
        if self.recording_data and self.prev_robot_on:
            self.data_storage.put_robot(curr_position, 
                                        np.zeros(3),
                                        t=self._current_recording_time - self._start_recording_time,
                                        laser_on=self.laser_obj.is_firing())
            
        if now - self._last_pose_ui < 1/75.0:  # 75 Hz
            return
        self._last_pose_ui = now
    
        warped = self.desired_state.isTransformedViewOn
        current_pixel_location = self.cam_reg.get_world_m_to_UI(self.cam_type, curr_position, warped)[0].astype(np.int16)
        
        self._track_virtual_fixtures(current_pixel_location)
        self.desired_state.laserX, self.desired_state.laserY = current_pixel_location
    
    
###------------------------ THERMAL OVERLAY -------------------####    

    def _do_current_thermal_info(self):
        if self.desired_state.heat_markers != None:
            if(len(self.desired_state.heat_markers) == 0 ):
                return
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
                    self.desired_state.heat_markers[i]["temp"] = float(temps[i])
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
    
    def _do_filter_prim_path(self, pixels):
        start_pixel, end_pixel = pixels[0, :], pixels[-1, :]
        epsilon = 2.0   # tolerance in pixels
        pixels_cv = np.asarray(pixels, dtype=np.float32).reshape(-1, 1, 2)
        approx = cv2.approxPolyDP(pixels_cv, epsilon, closed=True)
        poly_count = approx.shape[0]
        # print("approx corners", approx)
        if poly_count > 5:
            print("Circle")
            pixels = Motion_Planner.smooth_contour(pixels, window=31, poly=3)
            pixels = Motion_Planner.smooth_corners_fillet(pixels, radius=10, n_arc=20)
            pixels = np.vstack((np.vstack((start_pixel, pixels)), end_pixel))
        elif poly_count < 2: 
            print("line")
            pixels = np.vstack((start_pixel, end_pixel))
        else:
            print("Polygon")
            # pixels = np.vstack((np.vstack((start_pixel, pixels)), end_pixel))
            pixels = Motion_Planner.smooth_corners_fillet(pixels, radius=10, n_arc=20)
            # pixels = Motion_Planner.rdp(pixels, epsilon=4)
        return pixels
    
    def _do_create_path(self, path, raster=False):
        pixels = path
        path = None
        print("Path event?", self.desired_state.pathEvent)
        
        #TODO check if the path is wrapped, if not, do not include
        if raster:
            pixels = Motion_Planner.smooth_corners_fillet(pixels, radius=50, n_arc=20, min_angle=1e-4)
        else:
            # 
            pixels = self._do_filter_prim_path(pixels)
        
        fig, ax = plt.subplots(figsize=(8,4))
        # ax.imshow(img, cmap='gray')
        if len(pixels) > 1:
            xs = [p[0] for p in pixels]
            ys_plot = [p[1] for p in pixels]
            ax.plot(xs, ys_plot, linewidth=1)  # default color
        ax.set_axis_off()
        fig.savefig("plots/debug/smoothed pixels path.png")
        plt.close(fig)
        
        warped_view = self.desired_state.isTransformedViewOn
        # print("Warped Path: ", warped_view)
        robot_path = self.cam_reg.get_UI_to_world_m(
                self.cam_type, 
                pixels, 
                warped_view, 
                z = self.working_height)
        
        fig, ax = plt.subplots(figsize=(8,4))
        
        if len(robot_path) > 1:
            xs = [p[0] for p in robot_path]
            ys_plot = [p[1] for p in robot_path]
            ax.plot(xs, ys_plot, linewidth=1)  # default color
        ax.set_axis_off()
        fig.savefig("plots/debug/world points path.png")
        plt.close(fig)
            
        speed = self.desired_state.speed / 1000.0 if self.desired_state.speed != None else 0.01 # m/s
        traj = self.robot_controller.create_custom_trajectory(robot_path, speed)
        target_pixels, total_time = self._do_show_path(traj)
        self._package_path(target_pixels, total_time)
        return traj
    
    def _do_show_path(self, traj: TrajectoryController):
        target_positions = traj.get_path_position()
        # fig, ax = plt.subplots(figsize=(8,4))
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
        
        # plot pixels back
        # fig, ax = plt.subplots(figsize=(8,4), dpi=300)
        # if len(pixels) > 1:
        #     xs = [p[0] for p in pixels]
        #     ys_plot = [p[1] for p in pixels]
        #     ax.plot(xs, ys_plot, linewidth=1)  
        # ax.set_axis_off()
        # fig.savefig("pixels path.png")
        # plt.close()
        
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
        if self.hold_position is None:
            self.hold_position = self.robot_controller.current_robot_to_world_position()
            self.hold_pose, current_vel = self.robot_controller.get_current_state()
            # print("[Handler] Holding Pose: ", self.hold_position)
                
        current_pose, current_vel = self.robot_controller.get_current_state()
        # Stop robot, no drift
        # print(f"lin Norm: {np.linalg.norm(current_vel[:3]):0.4f}, rot Norm: {np.linalg.norm(current_vel[3:]):0.4f},")
        hold_orientation = self.robot_controller.hold_orientation
        if hold_orientation is not None:
            target_orien_vel = self.robot_controller.live_orientation_control(
                hold_orientation, 0.005, KP=0.05
            )

            if np.linalg.norm(target_orien_vel) < 1e-5:
                target_orien_vel = np.zeros(3)
        else:
            target_orien_vel = np.zeros(3)
            
        target_world_point = self.hold_position
        target_world_point[-1] = self.working_height
            
        target_pose = np.eye(4)
        target_pose[:3, -1] = target_world_point
        target_vel = self.robot_controller.live_control(target_pose, 0.01, KP = 0.1)
        
        if np.linalg.norm(target_vel) < 1e-4:
                target_vel = np.zeros(3)
            
        lin_cmd_norm = np.linalg.norm(target_vel)
        rot_cmd_norm = np.linalg.norm(target_orien_vel)

        lin_meas_norm = np.linalg.norm(current_vel[:3])
        rot_meas_norm = np.linalg.norm(current_vel[3:])
        
        # rot_error = hold_orientation

        not_home = lin_cmd_norm > 7e-5 or rot_cmd_norm > 3e-4
        not_stopped = lin_meas_norm > 2e-5 or rot_meas_norm > 1e-4

        # print(
        #     f"[DEBUG]\n"
        #     # f"  target_vel norm      = {lin_cmd_norm:.8e}  ({lin_cmd_norm > 7e-5})\n"
        #     f"  target_orien norm    = {rot_cmd_norm:.8e}  ({rot_cmd_norm > 3e-4})\n"
        #     # f"  current_lin norm     = {lin_meas_norm:.8e}  (>2e-5: {lin_meas_norm > 2e-5})\n"
        #     f"  rot error    = {rot_meas_norm:.8e}  (>2e-5: {rot_meas_norm > 1e-4})\n"
        #     # f"  not_home             = {not_home}\n"
        #     # f"  not_stopped          = {not_stopped}\n"
        # )
                
        if not_home or not_stopped:
            # print("[Handler] Hold pose: not home: ", not_home)
            # print("[Handler] Hold Pose: not stopped: ", not_stopped)
            self.robot_controller.set_velocity(target_vel, target_orien_vel)
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
        
        
        if(self._input_downtime() > .12 and np.abs(height_diff) < 0.00025): # 1mm
            # print("[Live Control] Holding Position")
            self._do_hold_pose()
        else:
            if self.desired_state.x is not None and self.desired_state.y is not None:
                if (self.desired_state.x < 0 and self.desired_state.y < 0):
                    return
                
                world_position = self.robot_controller.current_robot_to_world_position()
                valid_robot_position = self.robot_fixtures.is_valid(world_position[:2])
                
                pixel = np.array([[self.desired_state.x, self.desired_state.y]])
                
                if self.recording_data:
                    payload = {"pixel": pixel}
                    self.data_storage.put_user(t=self._current_recording_time - self._start_recording_time,
                                            mode = "live control",
                                            payload=payload)
                    print("[Handler Recorded Input] Live Control ", payload)
                
                warped_view = self.desired_state.isTransformedViewOn
                target_world_point = self.cam_reg.get_UI_to_world_m(
                    self.cam_type, 
                    pixel, 
                    warped_view, 
                    z = self.working_height)[0]
                
                valid_input_position = self.robot_fixtures.is_valid(target_world_point[:2])
                # print("[Hander] Robot Fixtures: Valid Input: ", valid_input_position, " | Valid Robot", valid_robot_position)
                if(not valid_input_position and not valid_robot_position):
                    # print("[Hander] Robot Fixtures: Stopping target", target_world_point[:2])
                    # print("[Hander] Robot Fixtures: Stopping current", world_position[:2])
                    self._do_hold_pose()
                    return
                    
                self.hold_position = None
                self.hold_pose = None
                if  height_diff < 0.0015: # dont fire if the height diff is too much
                    self.laser_obj.set_output(self.desired_state.isLaserOn)
                    # print("ready to fire height")
            else:
                # Holding but change height
                target_world_point = self.robot_controller.current_robot_to_world_position()
                target_world_point[-1] = self.working_height
            
                
            target_pose = np.eye(4)
            target_pose[:3, -1] = target_world_point
            live_control_speed = self.desired_state.speed / 1000.0 if self.desired_state.speed != None else 0.01
            # print(f"[Handler Live Control] Speed: {self.desired_state.speed}")
            target_vel = self.robot_controller.live_control(target_pose, live_control_speed, KP = 5.0, KD=0.1)
            # TODO Multiply velocity controller in unit component direction * max(min_speed, min(1, (distance / max_distance)))
            
            hold_orientation = self.robot_controller.hold_orientation
            if hold_orientation is not None:
                target_orien_vel = self.robot_controller.live_orientation_control(
                    hold_orientation, 0.005, KP=0.05
                )

                if np.linalg.norm(target_orien_vel) < 1e-4:
                    target_orien_vel = np.zeros(3)
            else:
                target_orien_vel = np.zeros(3)
            # print("here 3")
            self.robot_controller.set_velocity(target_vel, target_orien_vel)

###------------------------ Auto Laser Focus ------------------####

    def _do_auto_focus(self, world_pos):
        if self.cam_reg.depth_map is not None:
            current_height = DepthEstimation.current_height(depth_map = self.cam_reg.depth_map, 
                                                            current_position = world_pos[:2], 
                                                            meta = self.cam_reg.depth_meta)
            print(f"[Handler Depth Estimation] Current Height [mm]: {current_height*1000:0.2f}")
            # print("[Handler Depth Estimation] Current position [m]: ", world_pos[:2])

    async def main_loop(self):
        # Yield to other threads (video stream, websocket comms)
        await asyncio.sleep(0.0001)
        
        if(self.desired_state.isRobotOn != self.prev_robot_on):
            self._do_hold_pose() 
        
        self._do_current_thermal_info()
        # print(self.desired_state.heat_markers)
        self._do_current_position()
        
        if self.desired_state.isRecordingOn and not self.recording_data:
            self.recording_data = True
            self._start_recording_time = time.time()
            print("[Hander] Start Recording ")
            self.desired_state.isRecordingOn = None
        elif self.desired_state.isRecordingOn is not None and \
            not self.desired_state.isRecordingOn and self.recording_data:
            self.recording_data = False
            print("[Hander] Ending Recording ", self.desired_state.isRecordingOn)
            self.data_storage.save_data_storage(os.path.join(self.directory, "data_collection"))
        
        if self.recording_data:
            self._current_recording_time = time.time() 
        
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
                self.current_traj = self._do_create_path(raster, raster=True)
                
            if self.recording_data:
                payload = {"pixel points": raster,
                            "img": self.desired_state.raster_mask}
                
                self.data_storage.put_user(t=self._current_recording_time - self._start_recording_time,
                                            mode = "raster",
                                            payload=payload)
                print("[Handler Recorded Input] Raster Control ", raster)
            
            self.desired_state.raster_mask = None
            self.desired_state.path = None
                
        elif(self.desired_state.path is not None and len(self.desired_state.path) > 1
                and not self.robot_controller.is_trajectory_running()):
            print("Path Trigger")
            self.laser_obj.set_output(False)
            path = self._read_path()
            
            self.current_traj = self._do_create_path(path)
            
            if self.recording_data:
                payload = {"pixel points": self.desired_state.path}
                    
                self.data_storage.put_user(t=self._current_recording_time - self._start_recording_time,
                                            mode = "outline",
                                            payload=payload)
            
            self.desired_state.x = None
            self.desired_state.y = None
            self.desired_state.path = None
            
        world_pos = self.robot_controller.current_robot_to_world_position()
        world_z = world_pos[-1]
        self.desired_state.current_height = world_z

        if(self.desired_state.isRobotOn):          
            self.working_height = self.desired_state.height / 100.0 if self.desired_state.height else  0 # cm to m
            # self._do_auto_focus(world_pos)
            # print(f"[Robot Height] {self.working_height} m")
                
            # print("loop",
            # "raster?", self.desired_state.raster_mask is not None,
            # "path?", self.desired_state.path is not None,
            # "Path event?", self.desired_state.pathEvent,
            # "traj_running?", self.robot_controller.is_trajectory_running())
            height_diff = self.working_height - world_z
            height_change = np.abs(height_diff) > 0.0001 #0.1 mm
            # print(f"[Robot Height] Height Change: {height_change}")
            if(self.robot_controller.is_trajectory_running() and self.path_display_pixels is not None):
                self.cam_reg.display_path = True
                self.cam_reg.get_path(self.path_display_pixels)
            else:
                self.cam_reg.display_path = False
                # self.robot_controller.report_live_path()
                # self._do_hold_pose()
                # self.path_display_pixels = None
            
            if(self.desired_state.fixtures_mask is not None):
                self._read_fixtures()
                self.desired_state.fixtures_mask = None
            
            
            if(self.current_traj is not None and self.desired_state.executeCommand
                   and not self.robot_controller.is_trajectory_running()):
                print("Executing Path")
                if self.recording_data:
                    payload = {"traj":  self.current_traj}
                    self.data_storage.put_user(t=self._current_recording_time - self._start_recording_time,
                                            mode = "execute",
                                            payload=payload)
                self._do_path(self.current_traj)
                self.desired_state.executeCommand = None
                self.current_traj = None
                self.hold_position = None
                self.hold_pose = None
            
            elif(((self.desired_state.x is not None and self.desired_state.y is not None) or height_change)
                   and not self.robot_controller.is_trajectory_running()):
                # print("here just chaning hiehgt")
                    # print(f"Live controller trigger {self.desired_state.x}, {self.desired_state.y}")
                self._do_live_control(height_diff)
                self.desired_state.path = None
            elif(not self.robot_controller.is_trajectory_running()):
                self._do_hold_pose()
            # elif(not self.robot_controller.is_trajectory_running())
            # print("here")
        else:
            # print("here else")
            self._do_hold_pose()
            self.laser_obj.set_output(False)
                
        self.prev_robot_on = self.desired_state.isRobotOn
        