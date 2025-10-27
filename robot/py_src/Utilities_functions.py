import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))


from franka_client import FrankaClient
import numpy as np
import time, subprocess
from scipy.spatial.transform import Rotation



def loadHomePose(home_pose_path = "home_pose.csv"):
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

def HT_Inv(homogeneousPose):
    """
    Computes the inverse of a homogeneous transformation matrix
    """
    R = homogeneousPose[0:3, 0:3]
    t = homogeneousPose[0:3, 3]
    R_inv = R.T
    t_inv = -R_inv @ t
    return np.concatenate((np.concatenate((R_inv, t_inv.reshape(3, 1)), axis=1), [[0, 0, 0, 1]]), axis=0)
    
def alignRobot_input(homePose, robot_obj: FrankaClient):
    newPose = homePose.copy()
    goToPose(homePose,robot_obj=robot_obj)

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
            goToPose(newPose,robot_obj=robot_obj)

        except ValueError:
            print("Invalid input. Please enter three numeric values separated by space.")

    return newPose

def loadAndEditPose(robot_obj: FrankaClient, filePath = "home_pose.csv"):
    # Default robot starting location 
    homePose = loadHomePose(home_pose_path=filePath)
    
    robot_obj.send_pose(homePose,1) # Send robot to zero position
    homePose = alignRobot_input(homePose, robot_obj)
    
    np.savetxt("home_pose.csv", homePose, delimiter=",")
    print("Saving new homePose...")
    return homePose
  
def goToPose(pose, linTol = .05, rotTol = 0.05, robot_obj = FrankaClient(), maxIterations = 24):
    # tolerances are in mm and degrees 
    error, angleError = 10000, 10000
    currPose = robot_obj.send_pose(pose, 1)
    iterations = 0
    print("Moving to Pose ...")
    errorFlag1 = True
    errorFlag2 = True
    while (errorFlag1 or errorFlag2) and iterations < maxIterations:
        time.sleep(.5)
        currPose = robot_obj.send_pose(pose, 1)
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


if __name__ == '__main__':
    pathToCWD = os.getcwd()
    fileLocation = "python_src/thermal_cam_control/img_processing/LaserScanningExperiments/7-31-25/"

    subprocess.Popen([pathToCWD + "/cpp_src/main"]) 
    time.sleep(3)
    homePose = loadHomePose()
    goToPose(homePose)