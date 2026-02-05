import os, subprocess, time, sys, math
from scipy.spatial.transform import Rotation
import threading
import numpy as np
import matplotlib.pyplot as plt
from laser_control.laser_arduino import Laser_Arduino

if __name__=='__main__':    
    from franka_client import FrankaClient
    from controllers.trajectory_controller import TrajectoryController
else:
    from .franka_client import FrankaClient
    from .controllers.trajectory_controller import TrajectoryController

class Robot_Controller():
    def __init__(self, laser_obj:Laser_Arduino):
        pathToCWD = os.getcwd()
        self.franka_client = FrankaClient()
        self.laser_obj = laser_obj
        subprocess.Popen([pathToCWD + "/surgical_system/cpp_src/main"]) 
        time.sleep(3)
        print("Robot Online")
        self.home_pose = self.load_home_pose()
        self.home_pose_inv = self.HT_Inv(self.home_pose)
        
         # Trajectory thread management
        self._traj_thread = None
        self._stop_traj = threading.Event()

    def load_home_pose(self, home_pose_path = "surgical_system/py_src/robot/home_pose.csv"):
        if(os.path.exists(home_pose_path)):
            homePose = np.loadtxt(home_pose_path, delimiter=",")
            print("\nLoad Robot home pose: ", homePose)
        else:
            print("\n[WARNING] home_position.csv not found: Setting default home pose.")
            # Load home pose array
            rot = Rotation.from_euler('ZYX',[0,np.pi/4,np.pi/2])
            rotM = rot.as_matrix()
            
            #-75
            # Default robot starting location 
            homePosition = np.array([[0.5275],[0.0893],[0.3085]])
            homePose = np.concatenate((rotM,homePosition),axis=1)
            homePose = np.concatenate((homePose,[[0,0,0,1]]),axis=0)
            np.savetxt(home_pose_path, homePose, delimiter=",")
            print("Saving new home_pose...")
        return homePose
    
    def load_edit_pose(self, filePath = "surgical_system/py_src/robot/home_pose.csv"):
        # Default robot starting location 
        self.home_pose = self.load_home_pose(home_pose_path=filePath)
        self.franka_client.send_pose(self.home_pose,1) # Send robot to zero position
        self.home_pose = self.align_robot_input(self.home_pose)
        np.savetxt(filePath, self.home_pose, delimiter=",")
        print("Saving new self.home_pose...")
        return self.home_pose
  

    def go_to_pose(self, pose, linTol = .05, rotTol = 0.05, maxIterations = 24, blocking = True):
        # tolerances are in mm and degrees 
        error, angleError = 10000, 10000
        currPose = self.franka_client.send_pose(pose, 1)
        iterations = 0
        # print("Moving to Pose ...")
        errorFlag1 = True
        errorFlag2 = True
        while (errorFlag1 or errorFlag2) and iterations < maxIterations and blocking:
            time.sleep(.5)
            currPose = self.franka_client.send_pose(pose, 1)
            error = currPose[:3] - pose[:3, -1]
            
            # Convert to scipy Rotations
            rot_matrix = Rotation.from_matrix(pose[:3, :3])
            rot_quat = Rotation.from_quat(currPose[3:7])  # expects [x, y, z, w]

            # Compute relative rotation: R_rel = R_quat^-1 * R_matrix
            R_rel = rot_quat.inv() * rot_matrix

            # Get axis-angle representation
            rotvec = R_rel.as_rotvec()  # axis * angle in radians
            # axis = rotvec / np.linalg.norm(rotvec)
            
            angleError = np.linalg.norm(rotvec) #[rad]
            if not errorFlag1:
                errorFlag2 = (np.linalg.norm(error) > (linTol/1000) or np.linalg.norm(angleError) > np.deg2rad(rotTol))
                
            errorFlag1 = (np.linalg.norm(error) > (linTol/1000) or np.linalg.norm(angleError) > np.deg2rad(rotTol))

            iterations += 1
        #     sys.stdout.write(f"\r{(iterations*0.5):.1f} sec                \n")
        #     sys.stdout.write(f"Robot Position Error [mm]: [{1000*error[0]:.3f}, {1000*error[1]:.3f}, {1000*error[2]:.3f}]                  \n")
        #     sys.stdout.write(f"Robot Rotation Error [deg]: {np.rad2deg(angleError):.4f}         \033[F\033[F")
        #     sys.stdout.flush()

        # sys.stdout.write("\033[E\033[E\n...Done\n")
        # sys.stdout.flush()    
        # print("Error Flag 2 value: ", errorFlag2)
        # time.sleep(5)
        return currPose
    
    def set_velocity(self, lin_vel, ang_vel):
        return self.franka_client.send_velocity(lin_vel, ang_vel)
    
    def robot_stop(self):
        velocity = self.franka_client.send_velocity(np.array([0, 0, 0]), np.array([0, 0, 0]))
        time.sleep(1)
        return velocity
    
    def align_robot_input(self, new_pose):
            
        self.go_to_pose(new_pose)
        while True:
            user_input = input("Enter offset in X Y Z (mm), separated by space (blank to escape): ")
            
            if user_input == "":
                print("Exiting manual offset mode.")
                break
            try:
                x_mm, y_mm, z_mm = map(float, user_input.strip().split())
                offset_m = np.array([x_mm, y_mm, z_mm]) / 1000.0  # Convert to meters

                # Apply the offset
                new_pose[:3, 3] += offset_m
                print(f"Applying offset (m): {offset_m}")
                print(f"New position (m): {new_pose[:3, 3]}")
                self.go_to_pose(new_pose)

            except ValueError:
                print("Invalid input. Please enter three numeric values separated by space.")

        return new_pose

    def get_home_pose(self):
        return self.home_pose
    
    def close_robot(self):
        self.franka_client.close()

    def get_current_state(self):
        pose = self.franka_client.request_pose()
        position, quat = pose[:3], pose[3:7]
        Rmat = Rotation.from_quat(quat).as_matrix()
        new_pose = np.eye(4)
        new_pose[0:3, -1], new_pose[:3, :3] = position, Rmat
        current_vel = pose[7:]
        return new_pose, current_vel

    def quat_to_Mat(self, pose):
        #raw robot pose [x, y, z, qx, qy, qz, w]
        pose_M = np.eye(4)
        pose_M[:3, :3] = Rotation.from_quat(pose[3:]).as_matrix()
        pose_M[:3, -1] = pose[:3]
        return pose_M

    def HT_Inv(self, homogeneousPose):
        """
        Computes the inverse of a homogeneous transformation matrix
        """
        R = homogeneousPose[0:3, 0:3]
        t = homogeneousPose[0:3, 3]
        R_inv = R.T
        t_inv = -R_inv @ t
        return np.concatenate((np.concatenate((R_inv, t_inv.reshape(3, 1)), axis=1), [[0, 0, 0, 1]]), axis=0)

    def create_custom_trajectory(self, path, max_velocity):
        path_info = {"Positions": path, 
                    "Pattern": "Custom",
                    "Radius": -1,
                    "Passes": 1,
                    "MaxVelocity": max_velocity, # [m/s]
                    "MaxAcceleration": 0.0,#[m/s/s]
                    "Durations":[-1]} #time per step
        return TrajectoryController(path_info, debug=True)
    
    def create_trajectory(self, path_info):
        if not isinstance(path_info['Positions'], np.ndarray):
            path_info['Positions'] = np.array( path_info['Positions'])
        path_info['MaxVelocity'] = path_info['MaxVelocity'] / 100.0
        path_info['MaxAcceleration'] = path_info['MaxAcceleration'] / 100.0
        path_info['Radius'] = path_info['Radius'] / 100.0
        path_info['Positions'] = path_info['Positions'] / 100.0 #converts from cm to m
        return TrajectoryController(path_info, debug=True)
    
    def _run_trajectory_worker(self, traj: TrajectoryController, laser_on):
        total_time = traj.durations[-1]
        print("Path Duration: ", total_time)
        
        start_pos = traj.start_pos
        start_pose = np.eye(4)
        start_pose[:3, -1] = start_pos
        print("Heading to starting location")
        current_pose = np.array(self.go_to_pose(start_pose @ self.home_pose))
        
        time_step = 0.05
        elapsedTime = 0.0
        t = time.monotonic()

        target_vel = np.zeros(3)
        
        # record data
        data_num = int(math.ceil(total_time / time_step))
        actual_vel_list = np.empty((data_num, 3))
        target_vel_list = np.empty((data_num, 3))
        actual_pos_list = np.empty((data_num, 3))
        target_pos_list = np.empty((data_num, 3))
        i = 0
        try:
            while (elapsedTime < total_time) and (not self._stop_traj.is_set()):
                now = time.monotonic()
                self.laser_obj.set_output(laser_on)
                if (now - t) >= time_step:
                    elapsedTime += (now - t)
                    t = now

                    movement = traj.update(elapsedTime)
                    target_vel = movement['velocity']
                    target_pos = movement['position']

                    # print(f"Time: {elapsedTime:0,.2f}", "Target Vel: ", target_vel, f" | Target Pos: ", target_pos)
                    
                    target_pose = np.eye(4)
                    target_pose[:3, -1] = target_pos
                    velocity_correction = self.live_control(target_pose, 0.015, 5) #TODO CHANGE MAX SPEED
                    # print("Target Vel: ", target_vel, "Correction: ", velocity_correction)
                    target_vel += velocity_correction
                    state = self.set_velocity(target_vel, [0, 0, 0]) 
                    
                    
                    pos, vel = state[:3], state[7:10]

                    if i < data_num:
                        actual_vel_list[i, :] = vel[:3] if i != 0 else np.zeros(3)
                        target_vel_list[i, :] = target_vel
                        actual_pos_list[i, :] = pos[:3] - self.home_pose[:3, -1]
                        target_pos_list[i, :] = target_pos
                        i += 1

            # stop robot on exit (normal or stopped)
            self.laser_obj.set_output(False)
            self.robot_stop()
    
            # Optional: if you are okay plotting from a thread, keep this.
            # Otherwise, you can return the data instead and plot in main thread.
            # time_range = np.arange(0, total_time, time_step)

            # plt.figure(figsize=(10,6))
            # plt.plot(time_range, actual_vel_list[:, 0], label="actual x")
            # plt.plot(time_range, actual_vel_list[:, 1], label="actual y")
            # plt.plot(time_range, actual_vel_list[:, 2], label="actual z")
            # plt.plot(time_range, target_vel_list[:, 0], '--', label="target x")
            # plt.plot(time_range, target_vel_list[:, 1], '--', label="target y")
            # plt.plot(time_range, target_vel_list[:, 2], '--', label="target z")
            # plt.xlabel("Time [s]")
            # plt.ylabel("velocity [m/s]")
            # title = "Time vs velocity " + traj.pattern
            # plt.title(title)
            # plt.grid(True)
            # plt.legend()
            # plt.tight_layout()
            # plt.savefig("time_vs_velocity_" + traj.pattern + ".png", dpi=200)
            # plt.close()

            # plt.figure(figsize=(10,6))
            # plt.plot(time_range, actual_pos_list[:, 0], label="actual x")
            # plt.plot(time_range, actual_pos_list[:, 1], label="actual y")
            # plt.plot(time_range, actual_pos_list[:, 2], label="actual z")
            # plt.plot(time_range, target_pos_list[:, 0], '--', label="target x")
            # plt.plot(time_range, target_pos_list[:, 1], '--', label="target y")
            # plt.plot(time_range, target_pos_list[:, 2], '--', label="target z")
            # plt.xlabel("Time [s]")
            # plt.ylabel("position [m]")
            # title = "Time vs position " + traj.pattern
            # plt.title(title)
            # plt.grid(True)
            # plt.legend()
            # plt.tight_layout()
            # plt.savefig("time_vs_position_" + traj.pattern + ".png", dpi=200)
            # plt.close()
        finally:
            self._stop_traj.set()
            self._traj_thread = None
            print("traj done")
        # return [actual_vel_list,
        #         target_vel_list,
        #         actual_pos_list,
        #         target_pos_list]
    
    def run_trajectory(self, traj: TrajectoryController, blocking: bool = True, laser_on = False):
        """
        Run trajectory either blocking or in a background thread.
        """
        if blocking:
            # Old behavior
            self._stop_traj.clear()
            self._run_trajectory_worker(traj)
        else:
            # Non-blocking background thread
            if self._traj_thread is not None and self._traj_thread.is_alive():
                print("[WARNING] A trajectory is already running.")
                return

            self._stop_traj.clear()
            self._traj_thread = threading.Thread(
                target=self._run_trajectory_worker,
                args=(traj,laser_on),
                daemon=True,
            )
            self._traj_thread.start()
    
    
    
    def is_trajectory_running(self) -> bool:
        return (self._traj_thread is not None
            and self._traj_thread.is_alive())

        
    def stop_trajectory(self, wait: bool = False):
        """
        Signal the running trajectory to stop and optionally wait for it.
        """
        self._stop_traj.set()
        # ensure robot commanded to stop
        self.robot_stop()

        if wait and self._traj_thread is not None:
            self._traj_thread.join(timeout=2.0)
            
    def current_robot_to_world_position(self):
        current_pose, _ = self.get_current_state()
        return current_pose[:3, -1] - self.home_pose[:3, -1]
            
    def live_control(self, target_pose_home, max_vel, KP = 5.0, KD = 0.01):
        current_pose, current_velocity = self.get_current_state()
        current_pose = current_pose[:3, -1] - self.home_pose[:3, -1]
        position_error = target_pose_home[:3, -1] - current_pose
        vel_vector = np.array(current_velocity[:3])
        # print("[Live Controller] KD Control : ", vel_vector * KD)
        target_vel = position_error * KP - vel_vector * KD
        mag = np.linalg.norm(target_vel)
        
        if mag == 0:
            target_vel = np.zeros(3) 
        elif mag > max_vel:
            target_vel = target_vel * (max_vel / mag)
        return target_vel
            
    
    
if __name__=='__main__':
    import time, subprocess
    from scipy.spatial.transform import Rotation
    robot_controller = Robot_Controller()
    home_pose = robot_controller.get_home_pose()
    
    mode = 1
    height = 0.05 # m
    x = 0
    y = 0
    target_pose = np.array([[1,0,0,x],[0,1,0,y],[0,0,1,height],[0,0,0,1]])

    time.sleep(2)
    returnedPose = robot_controller.go_to_pose(target_pose@home_pose,mode)
    print("Finished Command")
