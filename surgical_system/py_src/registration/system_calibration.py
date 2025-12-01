import time, cv2, subprocess, os
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt
import warnings


import sys, pathlib
HERE = pathlib.Path(__file__).resolve().parent
for candidate in (HERE, HERE.parent, HERE.parent.parent):
    if (candidate / "robot").is_dir() and (candidate / "cameras").is_dir() and (candidate / "laser_control").is_dir():
        sys.path.insert(0, str(candidate))
        break
from robot.robot_controller import Robot_Controller
from cameras.thermal_cam import ThermalCam
from cameras.RGBD_cam import RGBD_Cam
from cameras.cam_calibration import CameraCalibration
from laser_control.laser_arduino import Laser_Arduino

    
    
class System_Calibration():
    def __init__(self, therm_cam: ThermalCam, rgbd_cam: RGBD_Cam, robot_controller: Robot_Controller, laser_controller:Laser_Arduino):
        self.pathToCWD = os.getcwd()
        self.directory = self.pathToCWD + r"/surgical_system/py_src/registration"
        
        self.laser_controller = laser_controller
        self.robot_controller = robot_controller
        self.therm_cam = therm_cam
        self.rgbd_cam = rgbd_cam
        
        self.rgbd_cali = CameraCalibration()
        self.therm_cali = CameraCalibration()
        
        self.rgb_M = None
        self.therm_M = None

        self.home_pose = robot_controller.get_home_pose()
    
    def reprojection_test(self, cam_type, M, gridShape = np.array([2, 6]),
                         laserDuration = .15, debug=False, height = 0.001):
        
        input("Press Enter to continue re-projection test.")
        targetPose = np.array([[1.0, 0, 0, 0],
                            [0, 1, 0, 0],
                            [0,0,1,height],
                            [0,0,0,1]])
        
        roi_height = 0.1
        roi_pose = np.array([[1.0, 0, 0, 0],
                            [0, 1, 0, 0],
                            [0,0,1,roi_height],
                            [0,0,0,1]])
        self.robot_controller.go_to_pose(roi_pose @ self.home_pose)
        rowROI, colROI = self.select_ROI(cam_type)
        
        xPoints = (np.linspace(colROI[0], colROI[1], gridShape[0]))
        yPoints = (np.linspace(rowROI[0], rowROI[1], gridShape[1]))
        xValues, yValues = np.meshgrid(xPoints, yPoints)
        
        cam_obj = self.get_cam_obj(cam_type)
        pix_Per_M = cam_obj.pix_Per_M
        
        startImage = self.get_cam_latest(cam_type)
        
        imgCount = gridShape[0] * gridShape[1]
        laserPixelPoints = np.empty((2, imgCount))
        if cam_type == "thermal":
            imageSet = np.empty((startImage.shape[0], startImage.shape[1], imgCount))
        elif cam_type == "color":
            imageSet = np.empty((startImage.shape[0], startImage.shape[1], startImage.shape[2], imgCount))
        else:
            raise(f"Wrong camera type: {cam_type}")
        
        img_points = np.vstack((xValues.flatten(), yValues.flatten())).T.reshape(-1, 2)
        proj = cv2.perspectiveTransform(img_points.reshape(-1,1,2).astype(np.float32), M).reshape(-1,2) / pix_Per_M

        # print(f"Projected Points: {proj}")
        print("\nFiring in 3 seconds ...")
        for i, (x, y) in enumerate(zip(proj[:, 0], proj[:, 1]), start=0):
            targetPose[0:3,3] = [x, y, height]
            print(f"\nMoving to position: {targetPose[0:3,3]}")
            self.robot_controller.go_to_pose(targetPose@self.home_pose)
            print("Firing...")
            if cam_type == "thermal":
                self.laser_controller.set_output(True)
                time.sleep(laserDuration)
                self.laser_controller.set_output(False)
            img = self.get_cam_latest(cam_type)
            
            if cam_type == "thermal":
                img = cv2.normalize(img, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)


            laserPixel = self.get_hot_pixel(img, method="Centroid")
            laserPixelPoints[:, i] = laserPixel
            if cam_type == "thermal":
                imageSet[:, :, i] = img
            elif cam_type == "color":
                imageSet[:, :, :, i] = img
            
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
            print("Warped Laser Spot Pixel: ", wLaserPixel, "Target Pixel: ", proj[i, :]*pix_Per_M, "Error: ", wLaserPixel - proj[i,:]*pix_Per_M)

            time.sleep(1)  

    def repeat_reprojection_test(self):
        reprojectionTestHeight = input("Input height for reprojection test or 'q' to quit: ")
        while reprojectionTestHeight != "q":
            targetHeight = float(reprojectionTestHeight)
            if targetHeight > 1: 
                # We should never have a target height over 1 meter, so pretend it was passed in mm by accident
                warnings.warn("Height seems to be in mm, dividing by 1000")
                targetHeight = targetHeight/1000
                # Raise robot arm in case we want to change the object for the reprojection tests

            targetPose = np.array([[1.0, 0, 0, 0],[0,1.0,0,0],[0,0,1,targetHeight],[0,0,0,1]])
            self.robot_controller.go_to_pose(targetPose@self.robot_controller.home_pose)
            laserDuration = targetHeight*7 + 0.15
            self.reprojection_test("thermal", self.therm_M, gridShape=[2, 2], height = targetHeight, laserDuration=laserDuration)
            self.reprojection_test("color", self.rgb_M, gridShape=[2, 2], height = 0.05, laserDuration=laserDuration)
            reprojectionTestHeight = input("Input height [m] for reprojection test or 'q' to quit: ")


    def pixel_to_world(self, img_points, cam_type, z = 0.0):
        M = self.get_cam_M(cam_type)
        cam_obj = self.get_cam_obj(cam_type)
        pix_Per_M = cam_obj.pix_Per_M
        world_point = np.zeros(3)
        world_point[-1] = z
        world_point[:2] = cv2.perspectiveTransform(img_points.reshape(-1,1,2).astype(np.float32), M).reshape(-1,2) / pix_Per_M
        return world_point
    
    def select_ROI(self, cam_type):
        print("\nSelecting Region of Interest")
        
        image = self.get_cam_latest(cam_type)
        if(cam_type == "thermal"):
            image = np.array(image*255.0/image.max(),dtype=np.uint8)
        bbox = cv2.selectROI('select', image)
        rowROI = [bbox[1], bbox[1]+bbox[3]]
        colROI = [bbox[0], bbox[0]+bbox[2]]
        cv2.destroyWindow('select')
        return rowROI, colROI
    
    def get_cam_M(self, cam_type):
        if cam_type == "color":
            return self.rgb_M
        elif cam_type == "thermal":
            return self.therm_M
        else:
            print("Incorrect camera type: ", cam_type)
            return None
    
    def get_cam_latest(self, cam_type):
        cam_obj = self.get_cam_obj(cam_type)
        return cam_obj.get_latest()[cam_type]
        
    def get_cam_obj(self, cam_type):
        if cam_type == "color" or cam_type =="depth":
            return self.rgbd_cam
        elif cam_type == "thermal":
            return self.therm_cam
        else:
            print("Incorrect camera type: ", cam_type)
            return None
    
    def get_cam_cali(self, cam_type = "color"):
        if cam_type == "color" or "depth":
            return self.rgbd_cali
        elif cam_type == "thermal":
            return self.therm_cali
        else:
            print("Incorrect camera type: ", cam_type)
            return None
    
    def get_hot_pixel(self, img, method="Centroid", thresholdScale= 0.8):
        
        if img.ndim == 3 and img.shape[-1] >= 3:
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        else:
            img = img
            
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
    

    
if __name__=='__main__':
    pass