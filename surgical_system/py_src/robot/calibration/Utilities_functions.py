import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))


from surgical_system.py_src.robot.franka_client import FrankaClient
from cameras.thermal_cam import ThermalCam
from laser_control.laser_arduino import Laser_Arduino
import numpy as np
import time, subprocess
import matplotlib.pyplot as plt
import cv2 
from scipy.spatial.transform import Rotation

def SelectROI(homePose, targetPose, cam_obj=None):
    print("\nSelecting Region of Interest")
    if cam_obj is None:
        cam_obj = ThermalCam(IRFormat="TemperatureLinear10mK", height=120)
        cam_obj.set_acquisition_mode()
    laser_obj = Laser_Arduino()
    
    print("Firing Laser in 2 seconds")
    robot_obj = FrankaClient()
    robot_obj.send_pose(targetPose@homePose,1)
    time.sleep(2)
    print("Laser On")
    laser_obj.set_output(1)
    time.sleep(0.5)
    laser_obj.set_output(0)
    print("Laser Off")
    image_list = cam_obj.acquire_and_display_images(5, display=False, debug=False)
    im = (image_list[4] - image_list[4].min())
    im = np.array(im*255.0/im.max(),dtype=np.uint8)
    bbox = cv2.selectROI('select',im)
    rowROI = [bbox[1], bbox[1]+bbox[3]]
    colROI = [bbox[0], bbox[0]+bbox[2]]
    cv2.destroyWindow('select')
    return rowROI, colROI

def manuallyFocusCamera(homePose, cam_obj = None):
    print("\nStart Manually Focusing the Camera\n")
    if cam_obj is None:
        cam_obj = ThermalCam(IRFormat="TemperatureLinear10mK", height=120)
        cam_obj.set_acquisition_mode()
    laser_obj = Laser_Arduino()
    print("Firing Laser in 2 seconds")
    robot_obj = FrankaClient()
    targetPose = np.array([1,0,0,0],[0,1,0,0],[0,0,1,0.15],[0,0,0,1])
    robot_obj.send_pose(targetPose@homePose,1)
    time.sleep(2)
    print("Laser On")
    laser_obj.set_output(1)
    cam_obj.acquire_and_display_images(500, display=True, debug=True)
    laser_obj.set_output(0)
    plt.close()
    print("Laser Off")

def HT_Inv(homogeneousPose):
    """
    Computes the inverse of a homogeneous transformation matrix
    """
    R = homogeneousPose[0:3, 0:3]
    t = homogeneousPose[0:3, 3]
    R_inv = R.T
    t_inv = -R_inv @ t
    return np.concatenate((np.concatenate((R_inv, t_inv.reshape(3, 1)), axis=1), [[0, 0, 0, 1]]), axis=0)
    
def alignRobot(homePose, robot_obj: FrankaClient):
    print("Robot Alignment (z). Use arrow keys. Enter = (x,y) alignment. ESC = exit.")
    newHomePose = homePose.copy()
    resolution = 0.001

    cv2.namedWindow("Keyboard Input Window")
    dummy_image = np.zeros((100, 400), dtype=np.uint8)
    cv2.imshow("Keyboard Input Window", dummy_image)
    # Z-alignment loop
    while True:
        key = cv2.waitKey(0) & 0xFF
        if key == 82:  # up arrow
            newHomePose[2, 3] += resolution
            print("↑ Pos Z:", newHomePose[2, 3])
        elif key == 84:  # down arrow
            newHomePose[2, 3] -= resolution
            print("↓ Neg Z:", newHomePose[2, 3])
        elif key == ord('z'):
            resolution = 0.0005
            print("Resolution 0.5 mm")
        elif key == ord('x'):
            resolution = 0.001
            print("Resolution 1 mm")
        elif key == 27:  # ESC
            print("Exiting Z mode.")
            break
        elif key == 13:  # Enter key to switch to Z mode
            print("Switching to X and Y alignment mode...")
            break

        robot_obj.send_pose(newHomePose, 1)
        
    while True:
        key = cv2.waitKey(0) & 0xFF  # Waits for key press

        if key == 81:  # left arrow
            newHomePose[0, 3] -= resolution
            print("← Neg X:", newHomePose[:2, 3])
        elif key == 82:  # up arrow
            newHomePose[1, 3] += resolution
            print("↑ Pos Y:", newHomePose[:2, 3])
        elif key == 83:  # right arrow
            newHomePose[0, 3] += resolution
            print("→ Pos X:", newHomePose[:2, 3])
        elif key == 84:  # down arrow
            newHomePose[1, 3] -= resolution
            print("↓ Neg Y:", newHomePose[:2, 3])
        elif key == ord('z'):
            resolution = 0.0005
            print("Resolution 0.5 mm")
        elif key == ord('x'):
            resolution = 0.001
            print("Resolution 1 mm")
        elif key == 27:  # ESC key to exit
            print("Exiting...")
            break
        robot_obj.send_pose(newHomePose, 1)
        
    cv2.destroyWindow("Keyboard Input Window")
    return newHomePose

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
    subprocess.Popen([pathToCWD + "/cpp_src/main"]) 
    time.sleep(3)
    homePose = loadHomePose()
    goToPose(homePose)