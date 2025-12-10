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

        initial_pose, _ = robot_controller.get_current_state()
        desired_state.update(asdict(RobotSchema.from_pose(initial_pose@np.linalg.inv(self.home_tf))))
        desired_state.isLaserOn = False
        desired_state.isRobotOn = True

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
        status = RobotSchema.from_pose(current_pose@np.linalg.inv(self.home_tf))
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

    def _do_raster(self):
        image_bytes = self.desired_state.raster_mask.encode('utf-8') # TODO have image available on shared disk mount
        numpy_array = np.frombuffer(image_bytes, np.uint8)
        cv2.imdecode(numpy_array, cv2.IMREAD_COLOR)
        print("recieved raster png")

    def _do_path(self):
        # TODO determine cam type from desired state
        # TODO convert path from List
        path = np.array([[d['x'], d['y']] for d in self.desired_state.path])
        robot_waypoints = self.cam_reg.pixel_to_world(path, cam_type=self.cam_type)
        traj = self.robot_controller.create_custom_trajectory(robot_waypoints, 0.01)
        self.robot_controller.run_trajectory(traj)

    def _do_live_control(self):
        # If no new message in 200ms, stop
        if(self._input_downtime() > .12):
            current_pose, current_vel = self.robot_controller.get_current_state()
            # Stop robot, no drift
            if np.linalg.norm(current_vel[:3]) > 2e-5:
                self.robot_controller.set_velocity(np.zeros(3), np.zeros(3))
            else:
                self.robot_controller.go_to_pose(current_pose, blocking=False)
                
            self.laser_obj.set_output(False)
        else:
            target_world_point = self.cam_reg.pixel_to_world(np.array([self.desired_state.x, self.desired_state.y]), cam_type=self.cam_type, z=self.start_pose[2,3])
            target_pose = np.eye(4)
            target_pose[:3, -1] = target_world_point
            target_vel = self.robot_controller.live_control(target_pose, 0.05)
            self.robot_controller.set_velocity(target_vel, np.zeros(3))

            self.laser_obj.set_output(self.desired_state.isLaserOn)

    async def main_loop(self):
        # Yield to other threads (video stream, websocket comms)
        await asyncio.sleep(0.0001)

        if(self.desired_state.isRobotOn):

            if(self.desired_state.raster_mask is not None):
                self._do_raster()
                self.desired_state.raster_mask = None
                
            elif(self.desired_state.path is not None and len(self.desired_state.path) > 1):
                self._do_path()
                self.desired_state.path = None

            else:
                self._do_live_control()
        