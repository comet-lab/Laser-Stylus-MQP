import time, cv2, subprocess, os
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt

if __name__=='__main__':
    import sys, pathlib
    HERE = pathlib.Path(__file__).resolve().parent
    # If this file is inside a subfolder, try parent too; stop at the first match.
    for candidate in (HERE, HERE.parent, HERE.parent.parent):
        if (candidate / "robot").is_dir() and (candidate / "cameras").is_dir() and (candidate / "laser_control").is_dir():
            sys.path.insert(0, str(candidate))
            break
        
    from robot.robot_controller import Robot_Controller
    from cameras.thermal_cam import ThermalCam
    from cameras.RGBD_cam import RGBD_Cam
    from cameras.cam_calibration import CameraCalibration
    from laser_control.laser_arduino import Laser_Arduino
    from system_calibration import System_Calibration
else:
    from ..robot.robot_controller import Robot_Controller
    from ..cameras.thermal_cam import ThermalCam
    from ..cameras.RGBD_cam import RGBD_Cam
    from ..cameras.cam_calibration import CameraCalibration
    from ..laser_control.laser_arduino import Laser_Arduino


# from Utilities_functions import SelectROI, loadAndEditPose, goToPose

class Camera_Registration(System_Calibration):
    def __init__(self, therm_cam: ThermalCam, rgbd_cam: RGBD_Cam, robot_controller: Robot_Controller, laser_controller:Laser_Arduino):
        super().__init__(therm_cam, rgbd_cam, robot_controller, laser_controller)
        self.calibration_folder = "calibration_info/"
        self.rgb_cali_folder = "calibration_info/rgb/"
        self.therm_cali_folder = "calibration_info/thermal/"
        
    def run(self):
        debug = True
        pathToCWD = os.getcwd()        
        homePose = self.robot_controller.loadAndEditPose()
        
        # Create checkerboard
        img, imgPoints, obj_points = self.createCheckerBoard(homePose, gridShape = np.array([9, 8]), \
                                                            saveLocation=self.calibration_folder, debug=debug)
        self.laser_controller.set_output(False)
        
        self.rgb_M = self.rgbd_cali.load_homography(fileLocation = self.rgb_cali_folder, debug = debug)
        self.therm = self.therm_cali.load_homography(fileLocation = self.therm_cali_folder, debug = debug)
        
        self.reprojection_test('color', gridShape = np.array([2, 2]), laserDuration = .15, \
                        debug=debug, height=0)
        
        self.reprojection_test('thermal', gridShape = np.array([2, 2]), laserDuration = .15, \
                        debug=debug, height=0)
        
        # cam_obj.deinitialize_cam()
        # pass
    
    def create_checkerboard(self, gridShape = np.array([2, 6]), squareSize = 0.005, laserDuration = .15, debug=False, \
                            saveLocation = "calibration_info/"):
        input("Press Enter to continue checkboard creation.")
    
        # Center the grid around 0
        xPoints = (np.arange(gridShape[1]) - (gridShape[1] - 1) / 2) * squareSize
        yPoints = (np.arange(gridShape[0]) - (gridShape[0] - 1) / 2) * squareSize
        xValues, yValues = np.meshgrid(xPoints, yPoints)
        
        therm_start_img = self.therm_cam.get_latest()['thermal']
        rgb_start_img = self.rgbd_cam.get_latest()['color']
        
        imgCount = gridShape[0] * gridShape[1]
        therm_img_points = np.empty((2, imgCount))
        rgb_img_points = np.empty((2, imgCount))
        
        therm_image_set = np.empty((therm_start_img.shape[0], therm_start_img.shape[1], imgCount))
        rgb_image_set = np.empty((rgb_start_img.shape[0], rgb_start_img.shape[1], imgCount))
        
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
            self.robot_controller.go_to_pose(targetPose@self.home_pose)
            print("Firing...")
            self.laser_controller.set_output(True)
            time.sleep(laserDuration)
            self.laser_controller.set_output(False)
            
            
            
            ### Get images ##
            therm_img = self.therm_cam.get_latest()
            color_img = self.rgbd_cam.get_latest()
            
            
            ## Thermal image pixel
            min_val = np.min(therm_img)
            max_val = np.max(therm_img)
            therm_img = (therm_img - min_val)/(max_val - min_val) * 255
            therm_img = cv2.normalize(therm_img, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
            therm_pixel_old = np.flip(np.asarray((np.unravel_index(np.argmax(therm_img), therm_img.shape))))
            therm_laser_pixel = self.hotSpotPixel(therm_img, method="Centroid")
            # print("Hottest pixel: ", laserPixelOld, "Centroid Pixel: ", laserPixel)
            
            ##rgbd image pixel
            rgb_laser_pixel = self.get_beam_pixel(color_img, method="Centroid", threshold= 0.8)
            
        
            therm_img_points[:, i] = therm_laser_pixel
            therm_image_set[:, :, i] = therm_img
            
            rgb_img_points[:, i] = rgb_laser_pixel
            rgb_image_set[:, :, i] = color_img
            
            if debug:
                cv2.imshow('Laser Spot', therm_img)
                cv2.waitKey(500)
                cv2.destroyAllWindows()
                # print(laserPixel)
            
            time.sleep(1)  
        combinedImg = np.sum(therm_image_set, axis=2)
        if debug:
            cv2.imshow('Combined Image', combinedImg)
            cv2.waitKey(1000)
            cv2.destroyAllWindows()
        
        
        therm_saveLocation = self.directory + saveLocation + "thermal_cali/"
        therm_save_dir = Path(therm_saveLocation)      
        therm_save_dir.mkdir(parents=True, exist_ok=True) 
        
        rgb_saveLocation = saveLocation + "rgb_cali/"
        rgb_saveLocation = Path(rgb_saveLocation)      
        rgb_saveLocation.mkdir(parents=True, exist_ok=True) 

        np.savetxt(therm_save_dir / "laser_spots.csv",        therm_img_points.T, delimiter=",")
        np.savetxt(therm_save_dir / "laser_world_points.csv", obj_Points,          delimiter=",")
        
        np.savetxt(rgb_saveLocation / "laser_spots.csv",        therm_img_points.T, delimiter=",")
        np.savetxt(rgb_saveLocation / "laser_world_points.csv", obj_Points,          delimiter=",")
        # np.save    (save_dir / "laser_spots.npy",       therm_image_set)
        return combinedImg, therm_img_points, obj_Points



if __name__ == '__main__':
    ##################################################################################
    #------------------------------ Robot Config ------------------------------------#
    ##################################################################################
    # Create FrankaNode object for controlling robot
    robot_controller = Robot_Controller()
    # TODO fix running in container
    home_pose = robot_controller.load_home_pose()
    start_pos = np.array([0,0,0.35]) # [m,m,m]
    target_pose = np.array([[1.0, 0, 0, start_pos[0]],
                            [0,1,0,start_pos[1]],
                            [0,0,1,start_pos[2]],
                            [0,0,0,1]])
    robot_controller.goToPose(target_pose@home_pose,1) # Send robot to start position
    time.sleep(2)
    
    ##################################################################################
    #--------------------------- Thermal Cam Config ---------------------------------#
    ##################################################################################
    # Set up camera object
    window_scale = 1
    frame_rate = 50  
    temp_scale = 100.0  # based on temperature linear 10mK reading
    # start with full window so we can perform camera calibration. Additionally, set maximum frame rate at 50 hz, 
    # and the focal distance to 0.204 m. This seems to be at the right location to maximize the focal point around the 
    # free beam laser spot.
    therm_cam = None
    therm_cam = ThermalCam(IRFormat="TemperatureLinear10mK", height=int(480/window_scale),frameRate="Rate50Hz",focalDistance=0.2) 
    
    ##################################################################################
    #------------------------------ RGBD Cam Config ---------------------------------#
    ##################################################################################
    rgbd_cam = RGBD_Cam() #Runs a thread internally

    
    ##################################################################################
    #-------------------------------- Laser Config ----------------------------------#
    ##################################################################################
    
    laser_controller = Laser_Arduino()  # controls whether laser is on or off
    laser_on = False
    laser_controller.set_output(laser_on)
    
    camera_reg = Camera_Registration(therm_cam, rgbd_cam, robot_controller, laser_controller)

