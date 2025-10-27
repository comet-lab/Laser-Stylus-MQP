import sys, os
import warnings
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from surgical_system.py_src.robot.franka_client import FrankaClient
from cameras.thermal_cam import ThermalCam
import subprocess, time, math
from cameras.thermal_camera_calibration import thermalCamCali
import numpy as np
from scipy.spatial.transform import Rotation
from laser_control.laser_arduino import Laser_Arduino
from Utilities.cameraRegistration import reProjectionTest, alignRobot_input, hotSpotPixel
from Utilities_functions import loadHomePose, goToPose
from pathlib import Path
import cv2

def laserAlignment():
    debug = True
    pathToCWD = os.getcwd()

    homePose = loadHomePose(home_pose_path="home_pose.csv")
    
    robot_obj = FrankaClient()  
    subprocess.Popen([pathToCWD + "/cpp_src/main"]) 
    time.sleep(3)
    goToPose(homePose,  robot_obj=robot_obj) # Send robot to zero position
    laser_obj = Laser_Arduino()  # controls whether laser is on or off

    # # Set up camera object
    windowScale = 1
    cam_obj = ThermalCam(IRFormat="TemperatureLinear10mK", height=int(480),frameRate="Rate50Hz",focalDistance=0.20) 
    cam_obj.set_acquisition_mode()
    
    fileLocation = "python_src/thermal_cam_control/img_processing/LaserScanningExperiments/"
    camCali = thermalCamCali(filepath=fileLocation)
    M_pixPerM = 7000
    camCali.load_homography(M_pix_per_m = M_pixPerM, fileLocation = fileLocation, debug = debug)
    M = camCali.M

    # Create a copy of the original home pose which we will edit in future functions   
    newHomePose = homePose.copy()
    
    newHomePose = xy_orientation_Correction(newHomePose, robot_obj, camCali, laser_obj, M_pixPerM)
    
    newHomePose, robotError = findRobotOffset(newHomePose, robot_obj, laser_obj, camCali,  M_pixPerM)
    
    reprojectionTestHeight = input("Input height for reprojection test or 'q' to quit: ")
    while reprojectionTestHeight != "q":
        targetHeight = float(reprojectionTestHeight)
        if targetHeight > 1: 
            # We should never have a target height over 1 meter, so pretend it was passed in mm by accident
            warnings.warn("Height seems to be in mm, dividing by 1000")
            targetHeight = targetHeight/1000
            # Raise robot arm in case we want to change the object for the reprojection tests

        targetPose = np.array([[1.0, 0, 0, 0],[0,1.0,0,0],[0,0,1,targetHeight],[0,0,0,1]])
        goToPose(targetPose@newHomePose,  robot_obj=robot_obj)
        laserDuration = targetHeight*7 + 0.15
        reProjectionTest(M, newHomePose, cam_obj, robot_obj, laser_obj, gridShape=[2, 2], M_pixPerM=M_pixPerM, \
                        height = targetHeight, laserDuration=laserDuration)
        reprojectionTestHeight = input("Input height [m] for reprojection test or 'q' to quit: ")

    newHomePose = alignRobot_input(newHomePose, robot_obj=robot_obj)

    saveLocation = fileLocation
    save_dir = Path(saveLocation)      
    save_dir.mkdir(parents=True, exist_ok=True) 
    np.savetxt('home_pose.csv', newHomePose, delimiter=',')
    print("Saved new home pose")
    homePose = loadHomePose(home_pose_path="home_pose.csv")
    cam_obj.deinitialize_cam()

    

def estimate_beam_orientation(laser_spots, target_z):
    # Perform least squares for alignment
    laser_spots = np.array(laser_spots) # [n x 3 array]
    # The height of the robot for each laser_spot
    target_z = np.array(target_z) # [ 1 x n array]
    # 
    # delta_zs =  target_z - laser_spots[:, 2] # [1 x n array]

    # each row in A represents [m, b] - slope and y-intercept of a line
    A = np.vstack([target_z, np.ones_like(target_z)]).T # [n x 2 array]
    
    # x-angle is defined by offset in y direction
    theta_x, y0 = np.linalg.lstsq(A, laser_spots[:, 1], rcond=None)[0]
    # y-angle is defined by offset in x direction
    theta_y, x0 = np.linalg.lstsq(A, laser_spots[:, 0], rcond=None)[0]
    
    # Create direction vector (θx, θy, 1), normalized
    d = np.array([theta_x, theta_y, 1.0])

    pitch_x = np.degrees(np.arctan2(d[0], d[2]))  # x tilt
    pitch_y = np.degrees(np.arctan2(d[1], d[2]))  # y tilt

    return {
        # "x0": x0,
        # "y0": y0,
        # "theta_x": theta_x,
        # "theta_y": theta_y,
        # "direction_vector": d,
        "pitch_x_deg": pitch_x, 
        "pitch_y_deg": pitch_y
    }
    
   
def xy_orientation_Correction(homePose, robot_obj, camCali, laser_obj, M_pixPerM):
    
    # send to initial pose to allow people to swap object
    targetPose = np.array([[1.0, 0, 0, 0],[0,1.0,0,0],[0,0,1,0.1],[0,0,0,1]])
    goToPose(targetPose@homePose,robot_obj=robot_obj)
    height = float(input("\nLaser-Robot Alignment: Enter max height [m]... "))
    imgCount = int(input("\nLaser-Robot Alignment: Enter num pulses... "))

    # [m] heights to test
    targetHeights = np.linspace(height, 0, imgCount)
    # initialize error
    xOrientationOffset, yOrientationOffset = 10, 10
    # make copy of homePose for edits
    _homePose = homePose.copy()
    
    while input("Press enter to perform an alignment (quit q): ") != "q":
        laserWorldPoints = np.empty((0, 3))
        # for each height target -> fire lase and get hot spot
        for i in range(len(targetHeights)):
            targetPose[2,3] = targetHeights[i]
            print("Moving to height: ", targetHeights[i])
            goToPose(targetPose@_homePose,robot_obj=robot_obj, linTol=0.025)
            print("Firing...\n")
            laser_obj.set_output(True)
            time.sleep(targetHeights[i]* 7 + 0.25) # always sleep for at least 0.15 seconds
            laser_obj.set_output(False)
            # Acquire image and find laser spot
            laserPixel = getPeakTemp(ThermalCam(), camCali) # pixel in world frame
            laserWorld = laserPixel / M_pixPerM # [m] in world frame
            print("Centroid in World: ",  laserWorld)
            laserWorldPoints = np.vstack((laserWorldPoints, laserWorld))
            time.sleep(1) # delay before next point

        # after we have our list of points, we can calculate the alignment
        alignment = estimate_beam_orientation(laserWorldPoints,targetHeights[:i+1])  # convert to [mm]
        print("Current offset: ", alignment)
            
        xOrientationOffset = -alignment["pitch_x_deg"]
        yOrientationOffset = alignment["pitch_y_deg"]
        
        print("Verification offset(x,y): ", xOrientationOffset, yOrientationOffset)
        print("Norm error: ", np.linalg.norm([xOrientationOffset,yOrientationOffset]))
        
        rot = Rotation.from_euler('XYZ', [xOrientationOffset, yOrientationOffset, 0],
                                degrees=True)
        rotM = rot.as_matrix()
        
        _homePose[:3, :3] = rotM @ _homePose[:3, :3]

        robot_obj.send_pose(_homePose,1)
        time.sleep(1)
        
        
    return _homePose

def findRobotOffset(newHomePose, robot_obj, laser_obj, camCali, M_pixPerM):
    height = float(input("\nLaser-Robot Alignment: Enter height for shift [m]... "))
    laserDuration = height* 7 + 0.25 # always sleep for at least 0.15 seconds
    target_px = np.array([0, 0])
    
    error = [1000, 1000]
    _newHomePose = newHomePose.copy()
    _newHomePose[2, -1] += height
    
    while input("Press enter to perform a shift (quit q): ") != "q":
        goToPose(_newHomePose,  robot_obj=robot_obj, linTol=0.025)
        print("Firing laser in 4 seconds...")
        fireLaser(laser_obj, duration=laserDuration)
        
        laser_px = getPeakTemp(ThermalCam(), camCali) # laser_px is [1x3]
        offset =  target_px - laser_px[0,0:2]
        error = np.append(offset,0) / M_pixPerM # px to m
        _newHomePose[:3, 3] += error
        print("Offset (mm): ", error*1000, "Norm Error (mm): ", np.linalg.norm(error)*1000)
        
    _newHomePose[2, -1] -= height
    return _newHomePose.copy(), np.linalg.norm(error*1000)
    
def fullRobotAlignment(robot_obj: FrankaClient, camCali: thermalCamCali, laser_obj: Laser_Arduino,\
                       homePose, M_pixPerM, height=0):
    robotError = 1000
    
    newHomePose = homePose.copy()
    while robotError > 0.2: # mm
        newHomePose = xy_orientation_Correction(newHomePose, \
                                robot_obj, camCali, laser_obj, M_pixPerM, height=height)
        
        # orientation_offset = old_orientation_offset # remove, only for debugging
        
        newHomePose, robotError = findRobotOffset(newHomePose, robot_obj, laser_obj, camCali,  M_pixPerM, height=height,  laserDuration = 1.5)
    

    goToPose(newHomePose, robot_obj=robot_obj)
    newHomePose = alignRobot_input(newHomePose, robot_obj)
    return newHomePose
    
def fireLaser(laser_obj, duration=0.5):
    print("Firing laser...")
    laser_obj.set_output(True)
    time.sleep(duration)
    laser_obj.set_output(False)
    print("Laser fired.")

def getPeakTemp(thermal_obj: ThermalCam, camCali: thermalCamCali, method="Centroid"):
    
    # Acquire image and find laser spot
    img = thermal_obj.acquire_and_display_images(1,display=False)[0]
    min_val = np.min(img)
    max_val = np.max(img)
    img = (img - min_val)/(max_val - min_val) * 255
    img = cv2.normalize(img, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    
    hottestPixel = hotSpotPixel(img, method=method)
    worldPoint = camCali.pixel_to_world(hottestPixel)
    camCali.change_image_perspective(img, marker=hottestPixel)
    return worldPoint


if __name__ == '__main__':
    laserAlignment()