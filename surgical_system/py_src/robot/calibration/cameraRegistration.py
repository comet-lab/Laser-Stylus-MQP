import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from robot.franka_client import FrankaClient
import time, cv2, subprocess
from pathlib import Path
from cameras.thermal_cam import ThermalCam
from cameras.thermal_camera_calibration import CameraCalibration
import numpy as np
from laser_control.laser_arduino import Laser_Arduino
from Utilities_functions import SelectROI, loadAndEditPose, goToPose
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt


def cameraRegistration(cam_obj: ThermalCam, robot_obj: FrankaClient, laser_obj:Laser_Arduino):
    debug = True
    pathToCWD = os.getcwd()
    fileLocation = "python_src/thermal_cam_control/img_processing/LaserScanningExperiments/"

    subprocess.Popen([pathToCWD + "/cpp_src/main"]) 
    time.sleep(3)
    
    laser_obj = Laser_Arduino()  # controls whether laser is on or off
    laser_obj.set_output(False)
    
    # # Set up camera object
    windowScale = 1
    cam_obj = ThermalCam(IRFormat="TemperatureLinear10mK", height=int(480/windowScale),frameRate="Rate50Hz",focalDistance=0.20) 
    cam_obj.set_acquisition_mode()
    camCali = CameraCalibration(filepath=fileLocation)
    
    homePose = loadAndEditPose(robot_obj)
    
    # Create checkerboard
    img, imgPoints, obj_points = createCheckerBoard(homePose, cam_obj, robot_obj, laser_obj, np.array([9, 8]),\
                                                    saveLocation=fileLocation, debug=debug)
    laser_obj.set_output(False)
    
    M_pixPerM = 7000
    
    camCali.load_homography(M_pix_per_m = M_pixPerM, fileLocation = fileLocation, debug = debug)
    M = camCali.M

    
    reProjectionTest(M, homePose, cam_obj, robot_obj, laser_obj, \
                     gridShape = np.array([2, 2]), M_pixPerM = M_pixPerM,laserDuration = .15, \
                     debug=debug, height=0)
    
    cam_obj.deinitialize_cam()
    

def createCheckerBoard(homePose, cam_obj: ThermalCam, robot_obj: FrankaClient, laser_obj: Laser_Arduino, \
                       gridShape = np.array([2, 6]), squareSize = 0.005, laserDuration = .15, debug=False, \
                       saveLocation = 'python_src/Utilities/'):
    
    input("Press Enter to continue checkboard creation.")
    
    # Center the grid around 0
    xPoints = (np.arange(gridShape[1]) - (gridShape[1] - 1) / 2) * squareSize
    yPoints = (np.arange(gridShape[0]) - (gridShape[0] - 1) / 2) * squareSize
    xValues, yValues = np.meshgrid(xPoints, yPoints)
    startImage = cam_obj.acquire_and_display_images(1,display=False)[0]
    
    
    imgCount = gridShape[0] * gridShape[1]
    laserPixelPoints = np.empty((2, imgCount))
    imageSet = np.empty((startImage.shape[0], startImage.shape[1], imgCount))
    
    
    targetPose = np.array([[1.0, 0, 0, 0],
                           [0, 1, 0, 0],
                           [0,0,1,0],
                           [0,0,0,1]])
    
    obj_Points = np.vstack((xValues.flatten(), yValues.flatten())).T.reshape(-1, 2)

    print("\nFiring in 4 seconds ...")
    for i, (x, y) in enumerate(zip(xValues.flatten(),
                               yValues.flatten()),    
                               start=0):
        targetPose[0:3,3] = [x, y, 0]
        print(f"\nMoving to position: {targetPose[0:3,3]}")
        goToPose(targetPose@homePose,  robot_obj=robot_obj)
        print("Firing...")
        laser_obj.set_output(True)
        time.sleep(laserDuration)
        laser_obj.set_output(False)
        img = cam_obj.acquire_and_display_images(1,display=False)[0]
        min_val = np.min(img)
        max_val = np.max(img)
        img = (img - min_val)/(max_val - min_val) * 255
        img = cv2.normalize(img, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        
        laserPixelOld = np.flip(np.asarray((np.unravel_index(np.argmax(img), img.shape))))
        laserPixel = hotSpotPixel(img, method="Centroid")
        # print("Hottest pixel: ", laserPixelOld, "Centroid Pixel: ", laserPixel)
        
        
        laserPixelPoints[:, i] = laserPixel
        imageSet[:, :, i] = img
        if debug:
            cv2.imshow('Laser Spot', img)
            cv2.waitKey(500)
            cv2.destroyAllWindows()
            # print(laserPixel)
        
        time.sleep(1)  
    combinedImg = np.sum(imageSet, axis=2)
    if debug:
        cv2.imshow('Combined Image', combinedImg)
        cv2.waitKey(1000)
        cv2.destroyAllWindows()
    
    
    save_dir = Path(saveLocation)      
    save_dir.mkdir(parents=True, exist_ok=True) 

    np.savetxt(save_dir / "laser_spots.csv",        laserPixelPoints.T, delimiter=",")
    np.savetxt(save_dir / "laser_world_points.csv", obj_Points,          delimiter=",")
    np.save    (save_dir / "laser_spots.npy",       imageSet)
    return combinedImg, laserPixelPoints, obj_Points
    
def reProjectionTest(M, homePose, cam_obj: ThermalCam, robot_obj: FrankaClient, laser_obj: Laser_Arduino,\
                    gridShape = np.array([2, 6]), M_pixPerM = 7000,laserDuration = .15, debug=False, \
                    height = 0.001):
    input("Press Enter to continue re-projection test.")
    
    targetPose = np.array([[1.0, 0, 0, 0],
                           [0, 1, 0, 0],
                           [0,0,1,height],
                           [0,0,0,1]])
    
    
    rowROI, colROI = SelectROI(homePose, targetPose, cam_obj)
    
    xPoints = (np.linspace(colROI[0], colROI[1], gridShape[0]))
    yPoints = (np.linspace(rowROI[0], rowROI[1], gridShape[1]))
    xValues, yValues = np.meshgrid(xPoints, yPoints)
    startImage = cam_obj.acquire_and_display_images(1,display=False)[0]
    
    
    imgCount = gridShape[0] * gridShape[1]
    laserPixelPoints = np.empty((2, imgCount))
    imageSet = np.empty((startImage.shape[0], startImage.shape[1], imgCount))
    
    img_points = np.vstack((xValues.flatten(), yValues.flatten())).T.reshape(-1, 2)
    proj = cv2.perspectiveTransform(img_points.reshape(-1,1,2).astype(np.float32), M).reshape(-1,2) / M_pixPerM

    # print(f"Projected Points: {proj}")
    print("\nFiring in 3 seconds ...")
    for i, (x, y) in enumerate(zip(proj[:, 0], proj[:, 1]), start=0):
        targetPose[0:3,3] = [x, y, height]
        print(f"\nMoving to position: {targetPose[0:3,3]}")
        goToPose(targetPose@homePose,  robot_obj=robot_obj)
        print("Firing...")
        laser_obj.set_output(True)
        time.sleep(laserDuration)
        laser_obj.set_output(False)
        img = CameraCalibration.get_thermal_image(cam_obj)
        img = cv2.normalize(img, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)

        laserPixel = hotSpotPixel(img, method="Centroid")
        laserPixelPoints[:, i] = laserPixel
        imageSet[:, :, i] = img
        
        print("Laser Error [pixels]: ", np.linalg.norm(laserPixel - img_points[i, :]))
        # Prepare a color image for drawing
        vis = img.copy()
        if vis.ndim == 2:
            vis = cv2.cvtColor(vis, cv2.COLOR_GRAY2BGR)

        h, w = vis.shape[:2]

        # Round to integer pixel coordinates and clip to image bounds
        lp = np.rint(laserPixel).astype(int)           # (x, y) laser pixel
        pp = np.rint(img_points[i, :]).astype(int)     # (x, y) projected pixel
        lp = (int(np.clip(lp[0], 0, w-1)), int(np.clip(lp[1], 0, h-1)))
        pp = (int(np.clip(pp[0], 0, w-1)), int(np.clip(pp[1], 0, h-1)))

        # Draw markers
        cv2.circle(vis, lp, 5, (0, 255, 0), 2)   # green: laser
        cv2.circle(vis, pp, 5, (0, 0, 255), 2)   # red: projected
        cv2.line(vis, pp, lp, (255, 0, 0), 1)    # blue line from projected -> laser
        
        cv2.imshow('Laser Spot', vis)
        cv2.waitKey(2000)
        cv2.destroyAllWindows()

        wLaserPixel = cv2.perspectiveTransform(laserPixel.reshape(-1,1,2), M).reshape(-1,2)
        print("Raw Laser Spot Pixel: ", laserPixel, "Target Pixel: ", img_points[i, :], "Error: ", laserPixel - img_points[i,:])
        print("Warped Laser Spot Pixel: ", wLaserPixel, "Target Pixel: ", proj[i, :]*M_pixPerM, "Error: ", wLaserPixel - proj[i,:]*M_pixPerM)

        time.sleep(1)  


def hotSpotPixel(img, method="Centroid", thresholdScale= 0.8):
    
    if method == "Centroid":
        peak = np.max(img)
        threshold = thresholdScale * peak
        mask = img >= threshold

        y, x = np.indices(img.shape)
        maskedImg = img * mask 
        x_center = np.sum(x * maskedImg) / np.sum(maskedImg)
        y_center = np.sum(y * maskedImg) / np.sum(maskedImg)
        
        
        # plt.imshow(maskedImg)
        # plt.show()
        center = np.array([x_center, y_center])
        
    else:
        center = np.flip(np.asarray((np.unravel_index(np.argmax(img), img.shape))))
    print("Raw Image Peak Temp Loc: ",np.flip(np.asarray((np.unravel_index(np.argmax(img), img.shape)))))
    print("Raw Image Center of centroid: ",center)
    
    return center

if __name__ == '__main__':
    # debug = True
    # pathToCWD = os.getcwd()
    # subprocess.Popen([pathToCWD + "/cpp_src/main"]) 
    # time.sleep(3)
    
    cameraRegistration(cam_obj = ThermalCam(), robot_obj = FrankaClient(),laser_obj = Laser_Arduino())
    # loadAndEditPose(FrankaClient())

