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
        self.rgbd_therm_cali = CameraCalibration()
        
        self.calibration_folder = "/calibration_info"
        self.rgb_cali_folder = "/calibration_info/rgb_cali/"
        self.therm_cali_folder = "/calibration_info/thermal_cali/"
        
        print("\n--------------------System Calibration-----------------")
        print("[System Calibration] RGBD Camera to robot calibration info:")
        self.rgb_M = self.rgbd_cali.load_homography(fileLocation = self.rgb_cali_folder)
        print("[System Calibration] Thermal Camera to robot calibration info:")
        self.therm_M = self.therm_cali.load_homography(fileLocation = self.therm_cali_folder)
        self.world_therm_M = np.linalg.inv(self.therm_M).astype(np.float32)
        
        img_points = CameraCalibration.load_pts(self.directory +  self.rgb_cali_folder + "laser_spots.csv")
        obj_points = CameraCalibration.load_pts(self.directory + self.therm_cali_folder  + "laser_spots.csv")
        print("[System Calibration]  RGBD camera to thermal camera calibration info:")
        self.rgbd_therm_M = self.rgbd_therm_cali.load_homography(M_pix_per_m = 1, img_points=img_points, obj_points=obj_points)
        
        
        self.rgb_H_shifted, self.rgb_out, self.rgb_min = self.make_positive_homography(self.rgb_M, (rgbd_cam.height, rgbd_cam.width))
        self.therm_H_shifted, self.therm_out, self.therm_min = self.make_positive_homography(self.therm_M, (therm_cam.height, therm_cam.width))
        
      
        

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


    def make_positive_homography(self, H, img_shape):
        """
        H: 3x3 homography (original -> warped), may produce negative coords
        img_shape: (h, w, ...) of the original image
        Returns:
            H_shifted: homography that maps original -> warped with all coords >= 0
            out_size: (out_w, out_h) to use with warpPerspective
        """
        H = H.astype(np.float32)
        h, w = img_shape[:2]

        # Original image corners
        src_corners = np.float32([
            [0, 0],
            [w - 1, 0],
            [w - 1, h - 1],
            [0, h - 1]
        ]).reshape(-1, 1, 2)
        

        # Transform corners with H
        dst_corners = cv2.perspectiveTransform(src_corners, H).reshape(-1, 2)

        xs = dst_corners[:, 0]
        ys = dst_corners[:, 1]

        x_min, x_max = xs.min(), xs.max()
        y_min, y_max = ys.min(), ys.max()

        # Output size is the size of the bounding box in warped space
        out_w = int(np.ceil(x_max - x_min))
        out_h = int(np.ceil(y_max - y_min))

        # Translation to shift everything so min coords become 0
        S = np.array([
            [1, 0, -x_min],
            [0, 1, -y_min],
            [0, 0, 1]
        ], dtype=np.float32)

        H_shifted = S @ H  
        
        #flip vertically 
        V = [[1,  0, 0], 
            [0, -1, out_h - 1],
            [0,  0,1]]

        H_shifted = V @ H_shifted
        
        return H_shifted, (out_w, out_h), (x_min, y_min)



    def world_to_pixel(self, world_points, cam_type):
        """
        Inverse of pixel_to_world for planar (x,y) points.

        Input:
            world_points: (N,2) or (N,3) array in world meters
        Output:
            img_points: (N,2) array in image pixels
        """
        
        cam_cali = self.get_cam_cali(cam_type)
        H_inv = cam_cali.M_inv
        pix_Per_M = self.get_cam_obj(cam_type).pix_Per_M

        world_points = np.asarray(world_points, dtype=np.float32)
        
        if world_points.ndim == 1:
            world_points = np.array([world_points])
        
        if world_points.shape[1] == 3:
            pts_m = world_points[:, :2]
        else:
            pts_m = world_points

        
        pts_wp = pts_m * pix_Per_M  # (N,2)

        img_pts = cv2.perspectiveTransform(
            pts_wp.reshape(-1, 1, 2).astype(np.float32),
            H_inv
        ).reshape(-1, 2)

        return img_pts
        
        
        

    def pixel_to_world(self, img_points, cam_type, z = 0.0):
        M = self.get_cam_M(cam_type)
        cam_obj = self.get_cam_obj(cam_type)
        pix_Per_M = cam_obj.pix_Per_M
        world_point = np.zeros((img_points.shape[0], 3))
        world_point[:, -1] = z
        world_point[:, :2] = cv2.perspectiveTransform(img_points.reshape(-1,1,2).astype(np.float32), M).reshape(-1,2) / pix_Per_M
        return world_point
    
    def world_to_real(self, img_points, cam_type, z = 0.0, pix_Per_M = None, display_w = 1280.0, display_h = 720.0):
        # 1) Get warped image size and the shift we used when building H_shifted
        out_w, out_h = self.get_cam_out(cam_type)      # (out_w, out_h)
        x_min, y_min = self.get_cam_min(cam_type)      # (x_min, y_min) from make_positive_homography

        pts = img_points.astype(np.float32).copy()
        
        pts[:, 0] = pts[:, 0] * (out_w / display_w)   # x: width scale
        pts[:, 1] = pts[:, 1] * (out_h / display_h)   # y: height scale

        # 2) Unflip (V^{-1} = V)
        pts[:, 1] = -pts[:, 1] + (out_h - 1)

        # 3) Unshift (T^{-1})
        pts[:, 0] = pts[:, 0] + x_min
        pts[:, 1] = pts[:, 1] + y_min

        cam_obj = self.get_cam_obj(cam_type)
        
        pix_Per_M = cam_obj.pix_Per_M if pix_Per_M == None else pix_Per_M
        

        world_point = np.zeros((pts.shape[0], 3), dtype=np.float32)
        world_point[:, :2] = pts / pix_Per_M
        world_point[:, 2] = z
        return world_point
    
    
    def real_to_world(self, world_points, cam_type,
                          pix_Per_M=None,
                          display_w=1280.0, display_h=720.0):
        """
        Inverse of world_to_real.

        Input:
            world_points: (N,2) or (N,3) array in WORLD METERS
        Output:
            display_points: (N,2) array in DISPLAY PIXELS
        """

        world_points = np.asarray(world_points, dtype=np.float32)

        if world_points.ndim == 1:
            world_points = np.array([world_points])
            
        if world_points.shape[1] == 3:
            pts = world_points[:, :2].copy()
        else:
            pts = world_points.copy()

        out_w, out_h = self.get_cam_out(cam_type)
        x_min, y_min = self.get_cam_min(cam_type)

        cam_obj = self.get_cam_obj(cam_type)
        pix_Per_M = cam_obj.pix_Per_M if pix_Per_M is None else pix_Per_M

        # 1) world meters → world pixels
        pts *= pix_Per_M

        # 2) undo translation
        pts[:, 0] -= x_min
        pts[:, 1] -= y_min

        # 3) undo flip
        pts[:, 1] = (out_h - 1) - pts[:, 1]

        # 4) undo scaling
        pts[:, 0] *= (display_w / out_w)
        pts[:, 1] *= (display_h / out_h)

        return pts
    
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
    
    def get_cam_out(self, cam_type):
        if cam_type == "color" :
            return self.rgb_out
        elif cam_type == "thermal":
            return self.therm_out
        else:
            print("Incorrect camera type: ", cam_type)
            return None
    
    
    def get_cam_min(self, cam_type):
        if cam_type == "color" :
            return self.rgb_min
        elif cam_type == "thermal":
            return self.therm_min
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