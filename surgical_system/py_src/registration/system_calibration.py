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
from registration.display_world_transform import Display_World_Transform

    
    
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
        self.rgbd_therm_cali = CameraCalibration()
        
        self.calibration_folder = "/calibration_info"
        self.rgb_cali_folder = "/calibration_info/rgb_cali/"
        self.therm_cali_folder = "/calibration_info/thermal_cali/"
        
        print("\n--------------------System Calibration-----------------")
        print("[System Calibration] RGBD Camera to robot calibration info:")
        self.cam_M = {}
        self.cam_M['color'] = self.rgbd_cali.load_homography(fileLocation = self.rgb_cali_folder)
        print("[System Calibration] Thermal Camera to robot calibration info:")
        self.cam_M['thermal'] = self.therm_cali.load_homography(fileLocation = self.therm_cali_folder)
        # self.world_therm_M = np.linalg.inv(self.cam_M['thermal']).astype(np.float32)
        
        img_points = CameraCalibration.load_pts(self.directory +  self.rgb_cali_folder + "laser_spots.csv")
        obj_points = CameraCalibration.load_pts(self.directory + self.therm_cali_folder  + "laser_spots.csv")
        print("[System Calibration]  RGBD camera to thermal camera calibration info:")
        self.rgbd_therm_M = self.rgbd_therm_cali.load_homography(M_pix_per_m = 1, img_points=img_points, obj_points=obj_points)
        
        
        self.cam_transforms = {}
        for cam in ("color", 'thermal'):
            cam_obj = self.get_cam_obj(cam)
            self.cam_transforms[cam] = Display_World_Transform( \
                H = self.cam_M[cam],
                img_shape = (cam_obj.height, cam_obj.width),
                pix_per_m = cam_obj.pix_Per_M,
                display_size = (cam_obj.height, cam_obj.width),
            )
        
        cam_obj = self.get_cam_obj(cam)
        self.cam_transforms["rgb_thermal"] = Display_World_Transform( \
                H = self.rgbd_therm_M,
                img_shape = (cam_obj.height, cam_obj.width),
                pix_per_m = 1,
                display_size = (cam_obj.height, cam_obj.width),
            )

        self.home_pose = robot_controller.get_home_pose()
    
    # --- Transformation Views --- #
    def get_transformed_view(self, img, cam_type = "color"):
        return self.cam_transforms[cam_type].warp_image_for_display(img)
    
    # --- UI Thermal <--> UI RGB
    def get_UI_to_thermal(self, points, warped):
        if warped:
            world_pixel = self.cam_transforms['color'].disp_px_to_world_m(points)
            return self.cam_transforms['thermal'].world_m_to_img_px(world_pixel).astype('int32')
        else:
            return self.cam_transforms['rgb_thermal'].img_px_to_world_m(points).astype('int32')
    
    
    
    # --- UI <--> world_m
    
    def get_UI_to_world_m(self, cam_type, points, warped, z = 0):
        if warped:
            return self.cam_transforms[cam_type].disp_px_to_world_m(points, z = z)
        else:
            return self.cam_transforms[cam_type].img_px_to_world_m(points, z = z)
        
    def get_world_m_to_UI(self, cam_type, world_points, warped):
        if warped:
            return self.cam_transforms[cam_type].world_m_to_disp_px(world_points)
        else:
            return self.cam_transforms[cam_type].world_m_to_img_px(world_points)

    # --- --- #
    
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
            self.reprojection_test("thermal", self.cam_M['thermal'], gridShape=[2, 2], height = targetHeight, laserDuration=laserDuration)
            self.reprojection_test("color", self.cam_M['color'], gridShape=[2, 2], height = 0.05, laserDuration=laserDuration)
            reprojectionTestHeight = input("Input height [m] for reprojection test or 'q' to quit: ")
    
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
    
    def draw_img(self, img, window_name="Paint Region"):
        """
        Let the user draw by click-dragging the mouse, recording only the cursor pixel.

        Controls:
        - Left-click + drag: record cursor positions
        - 'r': reset/clear
        - Enter: finish and return Nx2 np.array of (x, y) pixel coords
        - Esc: cancel, return empty array

        Returns
        -------
        coords : np.ndarray of shape (N, 2), dtype=int
            (x, y) pixel coordinates of all cursor positions while dragging.
        """
        drawing = False

        h, w = img.shape[:2]
        # Mask just for visualization
        mask = np.zeros((h, w), dtype=np.uint8)
        # List of recorded cursor positions
        coords_list = []

        def mouse_callback(event, x, y, flags, param):
            nonlocal drawing, mask, coords_list

            if event == cv2.EVENT_LBUTTONDOWN:
                drawing = True
                coords_list.append((x, y))
                mask[y, x] = 255

            elif event == cv2.EVENT_MOUSEMOVE and drawing:
                coords_list.append((x, y))
                mask[y, x] = 255

            elif event == cv2.EVENT_LBUTTONUP:
                drawing = False
                coords_list.append((x, y))
                mask[y, x] = 255

        cv2.namedWindow(window_name)
        cv2.setMouseCallback(window_name, mouse_callback)

        while True:
            disp = img.copy()
            if np.any(mask):
                overlay = disp.copy()
                # Color recorded pixels green
                overlay[mask == 255] = (0, 255, 0)
                alpha = 0.4
                disp = cv2.addWeighted(overlay, alpha, disp, 1 - alpha, 0)

            cv2.putText(disp, "Drag to record pixels | 'r' reset | Enter finish | Esc cancel",
                        (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (220, 220, 220), 1)

            cv2.imshow(window_name, disp)
            key = cv2.waitKey(20) & 0xFF

            if key in (13, 10):  # Enter
                break
            elif key == 27:  # Esc
                coords_list = []
                mask[:] = 0
                break
            elif key == ord('r'):
                coords_list = []
                mask[:] = 0

        cv2.destroyWindow(window_name)

        if not coords_list:
            return np.zeros((0, 2), dtype=int)

        # Convert list to array, optionally dedupe if you want unique pixels only
        coords = np.array(coords_list, dtype=int)

        # If you only want unique pixels:
        # coords = np.unique(coords, axis=0)

        return coords
    
    def moving_average_smooth(self, points, window=5):
        """
        Apply a simple moving average to (N, 2) points.
        Returns an array of the same length as `points`.
        """
        points = np.asarray(points, dtype=float)
        n = len(points)
        if n < 3 or window <= 1:
            return points.copy()

        # Do not let window be larger than the number of points
        window = min(window, n)

        # For even windows, distribute padding asymmetrically so that
        # we still end up with exactly n points after 'valid' convolution.
        pad_left  = window // 2
        pad_right = window - 1 - pad_left   # ensures pad_left + pad_right = window - 1

        # Pad endpoints to avoid shrinking the path
        padded = np.pad(points, ((pad_left, pad_right), (0, 0)), mode='edge')

        kernel = np.ones(window, dtype=float) / float(window)

        x_smooth = np.convolve(padded[:, 0], kernel, mode='valid')
        y_smooth = np.convolve(padded[:, 1], kernel, mode='valid')

        smoothed = np.stack((x_smooth, y_smooth), axis=1)

        # Safety: enforce same length as input
        if len(smoothed) > n:
            smoothed = smoothed[:n]
        elif len(smoothed) < n:
            # This should not happen with the padding logic above,
            # but just in case:
            smoothed = np.pad(smoothed, ((0, n - len(smoothed)), (0, 0)), mode='edge')

        smoothed[0] = points[0]
        smoothed[-1] = points[-1]

        return smoothed
    
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
    
    def get_cam_cali(self, cam_type="color"):
        if cam_type in ("color", "depth"):
            return self.rgbd_cali
        elif cam_type == "thermal":
            return self.therm_cali
        raise ValueError(f"Incorrect camera type: {cam_type}")
    
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