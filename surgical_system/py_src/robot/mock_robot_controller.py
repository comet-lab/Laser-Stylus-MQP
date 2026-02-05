import numpy as np
import time
from scipy.spatial.transform import Rotation
from laser_control.mock_laser import MockLaser

class MockRobotController():
    def __init__(self, laser_obj:MockLaser):
        self.pose = self.load_home_pose()
        self.current_velocity = [0, 0, 0]
        self.laser_obj = laser_obj
        self.last_update_time = time.time()
        self._is_trajectory_running = False

    def is_trajectory_running(self):
        return self._is_trajectory_running
        
    def go_to_pose(self, pose, linTol = .05):
        if(pose.shape != (4,4)):
            raise Exception("Incorrect pose shape")
        self.pose = pose
    
    def get_current_state(self):
        return self.pose, np.eye(4)
    
    def current_robot_to_world_position(self):
        return self.pose
    
    def load_home_pose(self):
        rot = Rotation.from_euler('ZYX',[0,np.pi/4,np.pi/2])
        rotM = rot.as_matrix()
        
        homePosition = np.array([[0.5275],[0.0893],[0.1985]])
        homePose = np.concatenate((rotM,homePosition),axis=1)
        homePose = np.concatenate((homePose,[[0,0,0,1]]),axis=0)
        # return homePose
        return np.eye(4)
    
    def set_velocity(self, lin_vel, ang_vel):
        return
        current_time = time.time()
        elapsed_time = current_time - self.last_update_time
        for i in range(3):
            self.pose[i, 3] += self.current_velocity[i] * elapsed_time
        self.current_velocity = lin_vel
        self.last_update_time = current_time

    def live_control(self, target_pose, max_vel, KP = 5.0, KD = 0.1):
        correction_position = target_pose - self.pose
        # correction_vector = correction_pose[0:3, 3]
        self.go_to_pose(target_pose)
    
    def create_custom_trajectory(self, robot_waypoints, speed):
        return (robot_waypoints, speed)

    def run_trajectory(self, traj, blocking, laser_on):
        print("Waypoint 0:", traj[0][0])
        print("Waypoint Final:", traj[0][-1])
        print("Speed", traj[1])
        self.go_to_pose(traj[0][-1])

