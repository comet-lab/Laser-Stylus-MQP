import os, subprocess, time, sys
from .franka_client import franka_client
from scipy.spatial.transform import Rotation
import numpy as np

class Robot_Controller():
    def __init__(self):
        pathToCWD = os.getcwd()
        self.franka_client = franka_client
        subprocess.Popen([pathToCWD + "surgical_system/cpp_src/main"]) 
        time.sleep(3)
        self.home_pose = self.load_home_pose()

    def load_home_pose(self, home_pose_path = "home_pose.csv"):
        if(os.path.exists(home_pose_path)):
            homePose = np.loadtxt(home_pose_path, delimiter=",")
            print("\nLoad Robot home pose: ", homePose)
        else:
            print("\n[WARNING] home_position.csv not found: Setting default home pose.")
            # Load home pose 
            rot = Rotation.from_euler('ZYX',[0,np.pi/4,np.pi/2])
            rotM = rot.as_matrix()
            
            # Default robot starting location 
            homePosition = np.array([[0.4425],[0.1043],[0.1985]])
            homePose = np.concatenate((rotM,homePosition),axis=1)
            homePose = np.concatenate((homePose,[[0,0,0,1]]),axis=0)

        return homePose
    
    def loadAndEditPose(self, filePath = "home_pose.csv"):
        # Default robot starting location 
        self.home_pose = self.load_home_pose(home_pose_path=filePath)
        self.franka_client.send_pose(self.home_pose,1) # Send robot to zero position
        self.home_pose = self.alignRobot_input()
        
        np.savetxt("home_pose.csv", self.home_pose, delimiter=",")
        print("Saving new self.home_pose...")
        return self.home_pose
  
    '''
    Code is blocking
    '''
    def goToPose(self, pose, linTol = .05, rotTol = 0.05, maxIterations = 24):
        # tolerances are in mm and degrees 
        error, angleError = 10000, 10000
        currPose = self.franka_client.send_pose(pose, 1)
        iterations = 0
        print("Moving to Pose ...")
        errorFlag1 = True
        errorFlag2 = True
        while (errorFlag1 or errorFlag2) and iterations < maxIterations:
            time.sleep(.5)
            currPose = franka_client.send_pose(pose, 1)
            error = currPose[:3] - pose[:3, -1]
            
            # Convert to scipy Rotations
            rot_matrix = Rotation.from_matrix(pose[:3, :3])
            rot_quat = Rotation.from_quat(currPose[3:])  # expects [x, y, z, w]

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
            sys.stdout.write(f"\r{(iterations*0.5):.1f} sec                \n")
            sys.stdout.write(f"Robot Position Error [mm]: [{1000*error[0]:.3f}, {1000*error[1]:.3f}, {1000*error[2]:.3f}]                  \n")
            sys.stdout.write(f"Robot Rotation Error [deg]: {np.rad2deg(angleError):.4f}         \033[F\033[F")
            sys.stdout.flush()

        sys.stdout.write("\033[E\033[E\n...Done\n")
        sys.stdout.flush()    
        print("Error Flag 2 value: ", errorFlag2)
        # time.sleep(5)
        return error, angleError
    
    def alignRobot_input(self):
        newPose = self.home_pose.copy()
        self.goToPose(self.home_pose)
        while True:
            user_input = input("Enter offset in X Y Z (mm), separated by space (blank to escape): ")
            
            if user_input == "":
                print("Exiting manual offset mode.")
                break
            try:
                x_mm, y_mm, z_mm = map(float, user_input.strip().split())
                offset_m = np.array([x_mm, y_mm, z_mm]) / 1000.0  # Convert to meters

                # Apply the offset
                newPose[:3, 3] += offset_m
                print(f"Applying offset (m): {offset_m}")
                print(f"New position (m): {newPose[:3, 3]}")
                self.goToPose(newPose)

            except ValueError:
                print("Invalid input. Please enter three numeric values separated by space.")

        return newPose

    def get_home_pose(self):
        return self.home_pose

if __name__=='__main__':
    import time, subprocess
    from scipy.spatial.transform import Rotation
    time.sleep(2)

    robot_controller = Robot_Controller()
    home_pose = robot_controller.get_home_pose()
    
    mode = 1
    height = 0.2 # m
    x = 0
    y = 0
    target_pose = np.array([[1,0,0,x],[0,1,0,y],[0,0,1,height],[0,0,0,1]])

    time.sleep(2)
    returnedPose = robot_controller.goToPose(target_pose@home_pose,mode)
    print("Finished Command")
