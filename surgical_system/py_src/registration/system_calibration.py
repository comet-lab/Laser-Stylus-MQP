import time, cv2, subprocess, os
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt

if __name__=='__main__':
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
else:
    from ..robot.robot_controller import Robot_Controller
    from ..cameras.thermal_cam import ThermalCam
    from ..cameras.RGBD_cam import RGBD_Cam
    from ..cameras.cam_calibration import CameraCalibration
    from ..laser_control.laser_arduino import Laser_Arduino
    
class System_Calibration():
    def __init__(self, therm_cam: ThermalCam, rgbd_cam: RGBD_Cam, robot_controller: Robot_Controller, laser_controller:Laser_Arduino):
        self.pathToCWD = os.getcwd()
        self.fileLocation = r"surgical_system\py_src\registration"
        
        self.laser_controller = laser_controller
        self.robot_controller = robot_controller
        self.therm_cam = therm_cam
        self.rgbd_cam = rgbd_cam
        
        self.rgbd_cali = CameraCalibration()
        self.therm_cali = CameraCalibration()

        self.home_pose = robot_controller.get_home_pose()
    
    def reprojection_test(self, cam_type, M, gridShape = np.array([2, 6]),/
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
        rowROI, colROI = self.SelectROI(targetPose, cam_type)
        
        xPoints = (np.linspace(colROI[0], colROI[1], gridShape[0]))
        yPoints = (np.linspace(rowROI[0], rowROI[1], gridShape[1]))
        xValues, yValues = np.meshgrid(xPoints, yPoints)
        
        cam_obj = self.get_cam_obj(cam_type)
        cam_cali = self.get_cam_cali(cam_type)
        pix_Per_M = cam_obj.pix_Per_M
        
        startImage = cam_obj.get_latest()
        startImage = startImage['thermal'] if cam_type is "thermal" else startImage["color"]
        
        imgCount = gridShape[0] * gridShape[1]
        laserPixelPoints = np.empty((2, imgCount))
        imageSet = np.empty((startImage.shape[0], startImage.shape[1], imgCount))
        
        img_points = np.vstack((xValues.flatten(), yValues.flatten())).T.reshape(-1, 2)
        proj = cv2.perspectiveTransform(img_points.reshape(-1,1,2).astype(np.float32), M).reshape(-1,2) / pix_Per_M

        # print(f"Projected Points: {proj}")
        print("\nFiring in 3 seconds ...")
        for i, (x, y) in enumerate(zip(proj[:, 0], proj[:, 1]), start=0):
            targetPose[0:3,3] = [x, y, height]
            print(f"\nMoving to position: {targetPose[0:3,3]}")
            self.robot_controller.go_to_pose(targetPose@self.home_pose)
            print("Firing...")
            self.laser_controller.set_output(True)
            time.sleep(laserDuration)
            self.laser_controller.set_output(False)
            img = CameraCalibration.get_thermal_image(cam_obj)
            img = cv2.normalize(img, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)

            if cam_type is "thermal":
                laserPixel = self.get_hot_pixel(img, method="Centroid")
            else:
                laserPixel = self.get_beam_pixel(img, method="Centroid")
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
            print("Warped Laser Spot Pixel: ", wLaserPixel, "Target Pixel: ", proj[i, :]*pix_Per_M, "Error: ", wLaserPixel - proj[i,:]*pix_Per_M)

            time.sleep(1)  

    def select_ROI(self, cam_type = "color"):
        print("\nSelecting Region of Interest")
        
        cam_obj = self.get_cam_obj(cam_type)
        
        # print("Firing Laser in 2 seconds")
        # self.robot_controller.goToPose(targetPose@self.home_pose,1)
        # time.sleep(2)
        # print("Laser On")
        # laser_controller.set_output(1)
        # time.sleep(0.5)
        # laser_controller.set_output(0)
        # print("Laser Off")
        
        image = cam_obj.get_latest()
        image = image['thermal'] if cam_type is "thermal" else image["color"]
        # im = (image_list[4] - image_list[4].min())
        # im = np.array(im*255.0/im.max(),dtype=np.uint8)
        bbox = cv2.selectROI('select', image)
        rowROI = [bbox[1], bbox[1]+bbox[3]]
        colROI = [bbox[0], bbox[0]+bbox[2]]
        cv2.destroyWindow('select')
        return rowROI, colROI
        
    def get_cam_obj(self, cam_type = "color"):
        if cam_type is "color" or "depth":
            return self.rgbd_cam
        elif cam_type is "thermal":
            return self.therm_cam
        else:
            print("Incorrect camera type: ", cam_type)
            return None
    
    def get_cam_cali(self, cam_type = "color"):
        if cam_type is "color" or "depth":
            return self.rgbd_cali
        elif cam_type is "thermal":
            return self.therm_cali
        else:
            print("Incorrect camera type: ", cam_type)
            return None
    
    def get_hot_pixel(self, img, method="Centroid", thresholdScale= 0.8):
    
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
    
    def get_beam_pixel(self, img, method="Centroid", threshold = 0.8):
        if img is None or img.size == 0:
            raise ValueError("get_beam_pixel: empty image input")

        # Ensure we have a 3-channel BGR image
        if img.ndim == 2:
            # grayscale -> fake BGR
            img_bgr = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        else:
            img_bgr = img

        # Split channels (OpenCV uses BGR)
        B, G, R = cv2.split(img_bgr.astype(np.int16))  # int16 to avoid underflow on subtraction

        # Redness response: R - max(G, B), clipped to [0, 255]
        GBmax = np.maximum(G, B)
        red_resp = (R - GBmax).clip(min=0).astype(np.float32)

        # If the dot is very bright/saturated, red_resp should be strongly peaked.
        rmax = float(red_resp.max())
        if rmax <= 1e-6:
            # No visible red; return image center
            h, w = img_bgr.shape[:2]
            return (w // 2, h // 2)

        # Try up to 3 threshold levels (requested `threshold`, then a bit looser)
        thresh_levels = [threshold, min(0.6, threshold), min(0.4, threshold)]
        mask = None

        for th in thresh_levels:
            th_val = th * rmax
            mask = (red_resp >= th_val).astype(np.uint8)  # {0,1}
            # Clean mask: small open + close to remove noise
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)
            if mask.sum() > 0:
                break

        # If still empty, fall back to the single most "red" pixel
        if mask is None or mask.sum() == 0:
            peak_idx = np.argmax(red_resp)
            h, w = red_resp.shape
            cy, cx = divmod(int(peak_idx), w)
            return (cx, cy)

        # Compute centroid (method-dependent)
        if method.lower() == "peak":
            # Choose the single peak within the current mask
            masked = red_resp * mask
            peak_idx = np.argmax(masked)
            h, w = masked.shape
            cy, cx = divmod(int(peak_idx), w)
            return (cx, cy)

        # Default: "Centroid" — weighted by red response within the mask
        # Use image moments (weighted by response) for subpixel centroid
        weighted = red_resp * mask
        m = cv2.moments(weighted, binaryImage=False)
        if abs(m["m00"]) < 1e-12:
            # Degenerate; fall back to peak
            masked = red_resp * mask
            peak_idx = np.argmax(masked)
            h, w = masked.shape
            cy, cx = divmod(int(peak_idx), w)
            return (cx, cy)

        cx = int(m["m10"] / m["m00"])
        cy = int(m["m01"] / m["m00"])
        return (cx, cy)
    
    
    
    
if __name__=='__main__':
    pass