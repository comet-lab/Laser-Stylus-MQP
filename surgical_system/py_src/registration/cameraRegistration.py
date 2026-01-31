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
    from system_calibration import System_Calibration
    from roi_selector import ROISelector
else:
    from registration.system_calibration import System_Calibration
    
from robot.robot_controller import Robot_Controller
from cameras.thermal_cam import ThermalCam
from cameras.RGBD_cam import RGBD_Cam
from cameras.cam_calibration import CameraCalibration
from laser_control.laser_arduino import Laser_Arduino


# from Utilities_functions import SelectROI, loadAndEditPose, go_to_pose

class Camera_Registration(System_Calibration):
    def __init__(self, therm_cam: ThermalCam, rgbd_cam: RGBD_Cam, robot_controller: Robot_Controller, laser_controller:Laser_Arduino):
        super().__init__(therm_cam, rgbd_cam, robot_controller, laser_controller)
        
        self.therm_cam.start_stream()
        self.rgbd_cam.start_stream()
        
        while not self.therm_cam.get_latest() or not self.rgbd_cam.get_latest():
            print("Waiting for camera response...")
            time.sleep(0.5)
            
        self.traj_points = None
        self.display_path = False
    
    @staticmethod
    def circle_perimeter_pixels(center, r, num_pts=360):
            cx, cy = center
            theta = np.linspace(0, 2*np.pi, num_pts, endpoint=False)
            xs = cx + r * np.cos(theta)
            ys = cy + r * np.sin(theta)
            return np.stack([xs, ys], axis=1).astype(np.float16)
        
    def run(self): 
        debug = True
       
        self.therm_cam.start_stream()
        self.rgbd_cam.start_stream()
        
        while not self.therm_cam.get_latest() or not self.rgbd_cam.get_latest():
            print("Waiting for camera response...")
            time.sleep(0.5)
        
        # pathToCWD = os.getcwd()        
        
        # Create checkerboard
        self.robot_controller.load_edit_pose()
        
        self.create_checkerboard(gridShape = np.array([9, 8]), \
                                 saveLocation=self.calibration_folder, debug=debug)
        
        self.laser_controller.set_output(False)
        
        
        self.reprojection_test('color', self.cam_M['color'], gridShape = np.array([2, 2]), laserDuration = .15, \
                        debug=debug, height=0)
        
        self.reprojection_test('thermal', self.cam_M['thermal'], gridShape = np.array([2, 2]), laserDuration = .15, \
                        debug=debug, height=0)
        
        self.laser_alignment()
        # self.therm_cam.deinitialize_cam()
        # pass
    
    
    def laser_alignment(self):
        # Create a copy of the original home pose which we will edit in future functions   
        new_home_pose = self.home_pose.copy()
        
        new_home_pose = self.xy_orientation_Correction()
    
        new_home_pose, robotError = self.findRobotOffset(new_home_pose)
        
        new_home_pose = self.robot_controller.align_robot_input(new_home_pose)
        self.robot_controller.home_pose = new_home_pose
        self.home_pose = new_home_pose

        self.repeat_reprojection_test()
        if input("Enter to save new home pose") == "":
            saveLocation = "surgical_system/py_src/robot/"
            save_dir = Path(saveLocation)      
            save_dir.mkdir(parents=True, exist_ok=True) 
            file_path = saveLocation + 'home_pose.csv'
            np.savetxt(file_path, new_home_pose, delimiter=',')
            print("Saved new home pose")
            home_pose = self.robot_controller.load_home_pose()
            self.robot_controller.go_to_pose(home_pose)
        
    def xy_orientation_Correction(self):
        # send to initial pose to allow people to swap object
        targetPose = np.array([[1.0, 0, 0, 0],[0,1.0,0,0],[0,0,1,0.1],[0,0,0,1]])
        self.robot_controller.go_to_pose(targetPose@self.home_pose)
        height = float(input("\nLaser-Robot Alignment: Enter max height [m]... "))
        imgCount = int(input("\nLaser-Robot Alignment: Enter num pulses... "))

        # [m] heights to test
        targetHeights = np.linspace(height, 0, imgCount)
        # initialize error
        xOrientationOffset, yOrientationOffset = 10, 10
        # make copy of homePose for edits
        _homePose = self.home_pose.copy()
        
        while input("Press enter to perform an alignment (quit q): ") != "q":
            laserWorldPoints = np.empty((0, 3))
            # for each height target -> fire lase and get hot spot
            for i in range(len(targetHeights)):
                targetPose[2,3] = targetHeights[i]
                print("Moving to height: ", targetHeights[i])
                self.robot_controller.go_to_pose(targetPose@_homePose, linTol=0.025)
                print("Firing...\n")
                self.laser_controller.set_output(True)
                time.sleep(targetHeights[i]* 7 + 0.25) # always sleep for at least 0.15 seconds
                self.laser_controller.set_output(False)
                # Acquire image and find laser spot
                
                therm_img = self.get_cam_latest("thermal")
                hottestPixel = self.get_hot_pixel(therm_img, method="Centroid") # pixel in world frame
                world_pix = self.therm_cali.pixel_to_world(hottestPixel)
                self.therm_cali.change_image_perspective(therm_img, marker=hottestPixel) # pixel in world frame
                laser_world = world_pix / self.therm_cam.pix_Per_M 
                print("Centroid in World: ",  laser_world)
                laserWorldPoints = np.vstack((laserWorldPoints, laser_world))
                time.sleep(1) # delay before next point

            # after we have our list of points, we can calculate the alignment
            alignment = self.estimate_beam_orientation(laserWorldPoints,targetHeights[:i+1])  # convert to [mm]
            print("Current offset: ", alignment)
                
            xOrientationOffset = -alignment["pitch_x_deg"]
            yOrientationOffset = alignment["pitch_y_deg"]
            
            print("Verification offset(x,y): ", xOrientationOffset, yOrientationOffset)
            print("Norm error: ", np.linalg.norm([xOrientationOffset,yOrientationOffset]))
            
            rot = Rotation.from_euler('XYZ', [xOrientationOffset, yOrientationOffset, 0],
                                    degrees=True)
            rotM = rot.as_matrix()
            
            _homePose[:3, :3] = rotM @ _homePose[:3, :3]

            self.robot_controller.go_to_pose(_homePose,1)
              
        return _homePose
    
    def estimate_beam_orientation(self, laser_spots, target_z):
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
    
    def findRobotOffset(self, newHomePose):
        height = float(input("\nLaser-Robot Alignment: Enter height for shift [m]... "))
        laserDuration = height* 7 + 0.25 # always sleep for at least 0.15 seconds
        target_px = np.array([0, 0])
        
        error = [1000, 1000]
        _newHomePose = newHomePose.copy()
        _newHomePose[2, -1] += height
        
        while input("Press enter to perform a shift (quit q): ") != "q":
            self.robot_controller.go_to_pose(_newHomePose, linTol=0.025)
            print("Firing laser in 4 seconds...")
            time.sleep(4)
            self.fireLaser(duration=laserDuration)
            
            therm_img = self.get_cam_latest("thermal")
            hottestPixel = self.get_hot_pixel(therm_img, method="Centroid") # pixel in world frame
            worldPoint = self.therm_cali.pixel_to_world(hottestPixel)
            self.therm_cali.change_image_perspective(therm_img, marker=hottestPixel)
            
            
            offset =  target_px - worldPoint[0,0:2]
            error = np.append(offset,0) / self.get_cam_obj("thermal").pix_Per_M # px to m
            _newHomePose[:3, 3] += error
            print("Offset (mm): ", error*1000, "Norm Error (mm): ", np.linalg.norm(error)*1000)
            
        _newHomePose[2, -1] -= height
        return _newHomePose.copy(), np.linalg.norm(error*1000)

    def fireLaser(self, duration=0.5):
        print("Firing laser...")
        self.laser_controller.set_output(True)
        time.sleep(duration)
        self.laser_controller.set_output(False)
        print("Laser fired.")
    
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
        rgb_image_set = np.empty((rgb_start_img.shape[0], rgb_start_img.shape[1], rgb_start_img.shape[2], imgCount))
        
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
            
            
            
            
            ### Get images ##
            therm_img = self.therm_cam.get_latest()['thermal']
            color_img = self.rgbd_cam.get_latest()['color']
            self.laser_controller.set_output(False)
            
            
            ## Thermal image pixel
            min_val = np.min(therm_img)
            max_val = np.max(therm_img)
            therm_img = (therm_img - min_val)/(max_val - min_val) * 255
            therm_img = cv2.normalize(therm_img, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
            therm_pixel_old = np.flip(np.asarray((np.unravel_index(np.argmax(therm_img), therm_img.shape))))
            therm_laser_pixel = self.get_hot_pixel(therm_img, method="Centroid")
            # print("Hottest pixel: ", laserPixelOld, "Centroid Pixel: ", laserPixel)
            ##rgbd image pixel
            rgb_laser_pixel = self.get_hot_pixel(color_img, method="Centroid")
            
        
            therm_img_points[:, i] = therm_laser_pixel
            therm_image_set[:, :, i] = therm_img
            
            rgb_img_points[:, i] = rgb_laser_pixel
            rgb_image_set[:, :, :, i] = color_img
            
            if debug:
                therm_img_copy = cv2.cvtColor(therm_img.copy(), cv2.COLOR_GRAY2BGR)
                color_img_copy = color_img.copy()
                cv2.circle(therm_img_copy, np.rint(therm_laser_pixel).astype(int), 5, (0, 255, 0), 2)   # green: laser
                cv2.circle(color_img_copy, np.rint(rgb_laser_pixel).astype(int), 5, (0, 255, 0), 2)   #green

                cv2.imshow('Thermal Laser Spot', therm_img_copy)
                cv2.waitKey(1000)
                cv2.imshow('color Laser Spot', color_img_copy)
                cv2.waitKey(1000)
                cv2.destroyAllWindows()
                # print(laserPixel)
            
            time.sleep(1)  
        combinedImg = np.sum(therm_image_set, axis=2)
        if debug:
            cv2.imshow('Combined Image', combinedImg)
            cv2.waitKey(1000)
            cv2.destroyAllWindows()
        
        
        therm_saveLocation = self.directory + saveLocation + "/thermal_cali/"
        therm_save_dir = Path(therm_saveLocation)      
        therm_save_dir.mkdir(parents=True, exist_ok=True) 
        
        rgb_saveLocation = self.directory + saveLocation + "/rgb_cali/"
        rgb_saveLocation = Path(rgb_saveLocation)      
        rgb_saveLocation.mkdir(parents=True, exist_ok=True) 

        np.savetxt(therm_save_dir / "laser_spots.csv",        therm_img_points.T, delimiter=",")
        np.savetxt(therm_save_dir / "laser_world_points.csv", obj_Points,          delimiter=",")
        
        np.savetxt(rgb_saveLocation / "laser_spots.csv",        rgb_img_points.T, delimiter=",")
        np.savetxt(rgb_saveLocation / "laser_world_points.csv", obj_Points,          delimiter=",")
        # np.save    (save_dir / "laser_spots.npy",       therm_image_set)
        return combinedImg, therm_img_points, rgb_img_points, obj_Points
    
    def get_path(self, points):
        self.traj_points = points
    
    def show_path(self, img, alpha=0.5):
        
        path = self.traj_points
        
        if path is None or len(path) < 2:
            return img

        out = img.copy()
        overlay = img.copy()

        pts = np.asarray(path, dtype=np.int32)

        # Optional: clip to image bounds to avoid weird drawing outside
        h, w = out.shape[:2]
        pts[:, 0] = np.clip(pts[:, 0], 0, w - 1)
        pts[:, 1] = np.clip(pts[:, 1], 0, h - 1)

        pts_i = np.round(pts).astype(np.int32).reshape(-1, 1, 2)

        # Draw polyline
        YELLOW = (0, 255, 255)
        GREEN = (0, 255, 0)
        BLACK = (0, 0, 0)
        cv2.polylines(overlay, [pts_i], isClosed=False, color=GREEN, thickness=1, lineType=cv2.LINE_AA)

        # Start marker (green circle)
        p0 = tuple(pts_i[0, 0])
        cv2.circle(overlay, p0, 3, (0, 255, 0), -1, lineType=cv2.LINE_AA)
        

        # End marker (red X)
        p1 = tuple(pts_i[-1, 0])
        cv2.drawMarker(overlay, p1, (0, 0, 255),
               markerType=cv2.MARKER_TILTED_CROSS,
               markerSize=5,
               thickness=2)
        
           
        
        cv2.arrowedLine(
            overlay,
            tuple(pts_i[0, 0]),
            tuple(pts_i[30, 0]),
            BLACK,
            1, #  thickness,
            cv2.LINE_AA,  
            0, # Shift 
            0.1 # Tip Length
        )
    
        # ---- Alpha blend overlay onto original ----
        cv2.addWeighted(overlay, alpha, out, 1 - alpha, 0, out)


        return out
    
    def heat_overlay(self, rgb_img, roi, transformed_view: bool = True,
                                alpha: float = 0.45,
                                colormap: int = cv2.COLORMAP_JET):

        def normalize_rect(x0, y0, x1, y1):
            xa, xb = sorted([x0, x1])
            ya, yb = sorted([y0, y1])
            return xa, ya, xb, yb
        
        therm_img = self.get_cam_latest('thermal')
        therm_h, therm_w = therm_img.shape[:2]
        disp = rgb_img.copy()
        vmin, vmax = None, None


        # If ROI exists, compute overlay inside ROI
        if roi is not None:
            x0, y0, x1, y1 = normalize_rect(*roi)
            x0, y0 = int(x0), int(y0)
            x1, y1 = int(x1), int(y1)
            
            cv2.rectangle(disp, (x0, y0), (x1, y1), (0, 255, 0), 1)

            # Create grid of display pixels in ROI
            xs = np.arange(x0, x1 + 1, dtype=np.float32)
            ys = np.arange(y0, y1 + 1, dtype=np.float32)
            X, Y = np.meshgrid(xs, ys)  # shapes (roi_h, roi_w)
            pts_disp = np.stack([X, Y], axis=-1).reshape(-1, 2)  # (N,2)

            if transformed_view:
                world_pts = self.cam_transforms['color'].disp_px_to_world_m(pts_disp)
                therm_uv = self.cam_transforms['thermal'].world_m_to_img_px(world_pts).astype('int32')
            else:
                therm_uv = self.cam_transforms['rgb_thermal'].img_px_to_world_m(pts_disp)[:, :2].astype('int32')

            # bounds checking
            tx = np.rint(therm_uv[:, 0]).astype(np.int32)
            ty = np.rint(therm_uv[:, 1]).astype(np.int32)

            valid = (tx >= 0) & (tx < therm_w) & (ty >= 0) & (ty < therm_h)

            # Prepare ROI heat buffer
            roi_h = (y1 - y0 + 1)
            roi_w = (x1 - x0 + 1)
            heat = np.zeros((roi_h * roi_w,), dtype=np.float32)
            heat[:] = np.nan

            # Fill valid samples
            heat[valid] = therm_img[ty[valid], tx[valid]].astype(np.float32)

            # Reshape to ROI image
            heat_img = heat.reshape(roi_h, roi_w)

            # Normalize heat for colormap (robust to NaNs)
            finite = np.isfinite(heat_img)
            if finite is not None and np.any(finite):
                vmin = np.nanmin(heat_img)
                vmax = np.nanmax(heat_img)
                if vmax - vmin < 1e-6:
                    norm = np.zeros_like(heat_img, dtype=np.uint8)
                else:
                    norm = np.zeros_like(heat_img, dtype=np.uint8)
                    norm[finite] = (255.0 * (heat_img[finite] - vmin) / (vmax - vmin)).astype(np.uint8)
            else:
                norm = np.zeros_like(heat_img, dtype=np.uint8)

            # Colorize
            heat_color = cv2.applyColorMap(norm, colormap)

            # mask out invalid pixels from overlay
            if np.any(~finite):
                heat_color[~finite] = 0

            # Alpha blend overlay 
            roi_bgr = disp[y0:y1 + 1, x0:x1 + 1]
            disp[y0:y1 + 1, x0:x1 + 1] = cv2.addWeighted(heat_color, alpha, roi_bgr, 1.0 - alpha, 0)
            return disp, finite, vmin, vmax
        return disp, None, None, None
    
    
    def tracking_display(self, disp, cam_type = 'color', warped = True):
        curr_position = self.robot_controller.current_robot_to_world_position()
        current_pixel_location = self.get_world_m_to_UI(cam_type, curr_position, warped)[0]
        current_pixel_location = np.asarray(current_pixel_location, dtype=np.int16)
        beam_waist = self.laser_controller.get_beam_width(curr_position[-1]) 

        laser_points = self.circle_perimeter_pixels(curr_position[:2], beam_waist/2.0)
        laser_pixels = self.get_world_m_to_UI(cam_type, laser_points, warped).astype(np.int16)

        xs = laser_pixels[:, 0]
        ys = laser_pixels[:, 1]
        h, w = disp.shape[:2]
        valid = (xs >= 0) & (xs < w) & (ys >= 0) & (ys < h)
        disp[ys[valid], xs[valid]] = (0, 255, 0)
        
        cv2.circle(disp, current_pixel_location, 5, (255, 0, 0), 2)
        return disp
### TESTING FUNCTIONS ###
    
    def transformed_view(self, cam_type = "color"):
        # Size of the top-down (bird's-eye) image (e.g., from calibration)
        WINDOW_NAME = "w0w"
        img = self.get_cam_latest(cam_type)

        warped = self.cam_transforms[cam_type].warp_image_for_display(img)
        

        selector = ROISelector(warped)
        cv2.namedWindow(WINDOW_NAME)
        cv2.setMouseCallback(WINDOW_NAME, selector.mouse_callback)

        while True:
            frame = selector.img.copy()
            selector.draw(frame)

            cv2.imshow(WINDOW_NAME, frame)

            # Show the ROI in a separate window
            roi = selector.get_roi()
            if roi is not None and roi.size > 0:
                cv2.imshow("wow", roi)

            key = cv2.waitKey(20) & 0xFF
            if key == 27:  # ESC to quit
                break

        cv2.destroyAllWindows()
        
    def live_control_view(self, cam_type, max_vel = 0.05, window_name="Camera", frame_key="color", warped = True, tracking = True):
        """
        Show a live view from cam_obj and allow the user to click to get pixel locations.
        """
        input("Press Enter to continue to live control")
        self.robot_controller.go_to_pose(self.home_pose)
        last_point = None   # np.array([x, y]) of the current/last selection
        dragging = False    # True while left mouse button is held

        def on_mouse(event, x, y, flags, param):
            nonlocal last_point, dragging

            if event == cv2.EVENT_LBUTTONDOWN:
                dragging = True
                last_point = np.array([x, y])
                # print(f"Pressed at pixel: (x={x}, y={y})")

            elif event == cv2.EVENT_MOUSEMOVE:
                if dragging and (flags & cv2.EVENT_FLAG_LBUTTON):
                    last_point = np.array([x, y])
                    # print(f"Dragging over pixel: (x={x}, y={y})")

            elif event == cv2.EVENT_LBUTTONUP:
                dragging = False
                last_point = np.array([x, y])
                # print(f"Released at pixel: (x={x}, y={y})")

        cv2.namedWindow(window_name)
        cv2.setMouseCallback(window_name, on_mouse)
        
        working_height = 0
        
        try:
            while True:
                
                frame_data = self.get_cam_latest(cam_type)

                # Handle either dict or raw image
                if isinstance(frame_data, dict):
                    frame = frame_data.get(frame_key, None)
                else:
                    frame = frame_data

                if frame is None:
                    continue

                disp = frame.copy()
                
                if warped:
                    # Test as if it was the UI 
                    disp = self.get_transformed_view(disp)
                    disp = cv2.resize(disp, (1280, 720), interpolation=cv2.INTER_NEAREST)
                    
                if tracking:
                    curr_position = self.robot_controller.current_robot_to_world_position()
                    current_pixel_location = self.get_world_m_to_UI(cam_type, curr_position, warped)[0]
                    current_pixel_location = np.asarray(current_pixel_location, dtype=np.int16)
                    beam_waist = self.laser_controller.get_beam_width(curr_position[-1]) 

                    laser_points = circle_perimeter_pixels(curr_position[:2], beam_waist/2.0)
                    laser_pixels = self.get_world_m_to_UI(cam_type, laser_points, warped).astype(np.int16)

                    xs = laser_pixels[:, 0]
                    ys = laser_pixels[:, 1]
                    h, w = disp.shape[:2]
                    valid = (xs >= 0) & (xs < w) & (ys >= 0) & (ys < h)
                    disp[ys[valid], xs[valid]] = (0, 255, 0)
                    
                    cv2.circle(disp, current_pixel_location, 5, (255, 0, 0), 2)

                if last_point is not None:
                    cv2.circle(disp, last_point, 5, (0, 255, 0), 2)
                    target_position = self.get_UI_to_world_m(cam_type, last_point, warped, z = working_height)[0]

                
                if dragging and last_point is not None:
                    target_pose = np.eye(4)
                    target_pose[:3, -1] = target_position
                    target_vel = self.robot_controller.live_control(target_pose, max_vel)
                    self.robot_controller.set_velocity(target_vel, np.zeros(3))
                    self.laser_controller.set_output(1)
                else:
                    current_pose, current_vel = self.robot_controller.get_current_state()
                    self.laser_controller.set_output(0)
                    # print(np.linalg.norm(current_vel[:3]))
                    if np.linalg.norm(current_vel[:3]) > 2e-5:
                        self.robot_controller.set_velocity(np.zeros(3), np.zeros(3))
                    else:
                        robot_controller.go_to_pose(current_pose, blocking=False)

                # Press 'q' or ESC to quit
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == 27:  # q or ESC
                    break
                elif key == 82:  # Up arrow
                    working_height += 0.005
                elif key == 84:  # Down arrow
                    working_height -= 0.005
                    
                # ----- UI text overlay -----
                x0, y0 = 15, 25
                line_h = 22
                font = cv2.FONT_HERSHEY_SIMPLEX
                scale = 0.55
                color = (255, 255, 255)
                thickness = 1

                cv2.putText(disp, "Controls:", (x0, y0),
                            font, scale, color, thickness, cv2.LINE_AA)

                cv2.putText(disp, "Up Arrow    : Increase working height (+0.01 m)",
                            (x0, y0 + line_h),
                            font, scale, color, thickness, cv2.LINE_AA)

                cv2.putText(disp, "Down Arrow  : Decrease working height (-0.01 m)",
                            (x0, y0 + 2*line_h),
                            font, scale, color, thickness, cv2.LINE_AA)

                cv2.putText(disp, "q / ESC     : Quit",
                            (x0, y0 + 3*line_h),
                            font, scale, color, thickness, cv2.LINE_AA)
                    
                cv2.imshow(window_name, disp)

        finally:
            cv2.destroyWindow(window_name)
            self.laser_controller.set_output(0)
    
    def draw_traj(self, cam_type = 'color'):
        img = self.get_cam_latest(cam_type)
        pixels = self.draw_img(img)
        pixels = self.moving_average_smooth(pixels, window=5)
        robot_path = self.pixel_to_world(pixels, cam_type)
        plt.plot(robot_path[:, 0], robot_path[:, 1])
        plt.show()
        traj = self.robot_controller.create_custom_trajectory(robot_path, 0.005)
        self.robot_controller.run_trajectory(traj)
        # print(pixels)
    
    def rgb_thermal_data(self, rgb_img, therm_img, pixel_point, transformed_view):
        disp = rgb_img.copy()
        disp = cv2.resize(disp, (1280, 720), interpolation=cv2.INTER_NEAREST)
        therm_h, therm_w = therm_img.shape[:2]
        temp = None 
        
        if pixel_point is not None:
            if transformed_view == True:
                mouse_loc = np.array([pixel_point], dtype=np.float32)
                world_loc = self.world_to_real(mouse_loc, "color", pix_Per_M = 1)[0, :2]
                pixel_loc = cv2.perspectiveTransform(np.array([[world_loc]]), 
                                                        self.world_therm_M).reshape((2)).astype('int32')
            else:
                pixel_loc = self.rgbd_therm_cali.pixel_to_world(pixel_point)[0][:2].astype('int32')
            
            # print(f"Pixel Location {world_loc}")
            if 0 <= pixel_loc[0] < therm_w and 0 <= pixel_loc[1] < therm_h:
                temp = therm_img[pixel_loc[1], pixel_loc[0]]
        
        return temp
    
    def view_rgbd_therm_registration(self, transformed_view = True):
        debug = True
       
        self.therm_cam.start_stream()
        self.rgbd_cam.start_stream()
        
        while not self.therm_cam.get_latest() or not self.rgbd_cam.get_latest():
            print("Waiting for camera response...")
            time.sleep(0.5)
        
        mouse_pos = None
        window_name = "wow this is cool"
        def on_mouse(event, x, y, flags, param):
            nonlocal mouse_pos
            if event == cv2.EVENT_MOUSEMOVE:
                mouse_pos = np.array([x, y])
                
        cv2.namedWindow(window_name)
        cv2.setMouseCallback(window_name, on_mouse)
        
        therm_img = self.get_cam_latest('thermal')
        therm_h, therm_w = therm_img.shape[:2]
        
        try:
            while True:
                
                rgb_img = self.get_cam_latest('color')
                therm_img = self.get_cam_latest('thermal')
                
                if rgb_img is None or therm_img is None:
                    continue
                
                
                if transformed_view == True:
                    rgb_img = self.get_transformed_view(rgb_img, cam_type="color")


                disp = rgb_img.copy()
                disp = cv2.resize(disp, (1280, 720), interpolation=cv2.INTER_NEAREST)
                
                if mouse_pos is not None:
                    pixel_loc = self.get_UI_to_thermal(mouse_pos, transformed_view)[0]
                    
                    # print(f"Pixel Location {world_loc}")
                    if 0 <= pixel_loc[0] < therm_w and 0 <= pixel_loc[1] < therm_h:
                        temp = therm_img[pixel_loc[1], pixel_loc[0]]
                        # print(f"Temperature: {temp}")
                
                        x, y = mouse_pos
                        
                        info = [
                            f"Temp: {temp:.2f} C",
                            f"x: {x}, y: {y}",
                        ]

                        for i, line in enumerate(info):
                            cv2.putText(
                                disp,
                                line,
                                (x, y + 20*i),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.5,
                                (0, 255, 0),
                                1,
                                cv2.LINE_AA
                            )

                cv2.imshow(window_name, disp)
                
                # Press 'q' or ESC to quit
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == 27:
                    break
                
        finally:
            cv2.destroyWindow(window_name)
            self.laser_controller.set_output(0)
        pass 

    def view_rgbd_therm_heat_overlay(self, transformed_view: bool = True,
                                alpha: float = 0.45,
                                colormap: int = cv2.COLORMAP_JET,
                                window_name: str = "RGB + Thermal Overlay"):
        """
        Display RGB with a thermal heat overlay inside a user-selected rectangle.

        Controls
        --------
        - Click + drag LEFT mouse to draw/resize rectangle ROI
        - Release to "lock" ROI (overlay updates live within it)
        - Press 'r' to reset ROI
        - Press 'q' or ESC to quit
        """

        self.therm_cam.start_stream()
        self.rgbd_cam.start_stream()

        while not self.therm_cam.get_latest() or not self.rgbd_cam.get_latest():
            print("Waiting for camera response...")
            time.sleep(0.5)

        # Display size (matches your existing function)
        DISP_W, DISP_H = 1280, 720

        # ROI state
        roi = None  # (x0, y0, x1, y1) in display coordinates
        dragging = False
        anchor = None  # (x0, y0)

        def clamp(v, lo, hi):
            return max(lo, min(hi, v))

        def normalize_rect(x0, y0, x1, y1):
            xa, xb = sorted([x0, x1])
            ya, yb = sorted([y0, y1])
            return xa, ya, xb, yb

        def on_mouse(event, x, y, flags, param):
            nonlocal roi, dragging, anchor
            if event == cv2.EVENT_LBUTTONDOWN:
                dragging = True
                anchor = (x, y)
                roi = (x, y, x, y)

            elif event == cv2.EVENT_MOUSEMOVE and dragging:
                x0, y0 = anchor
                roi = (x0, y0, x, y)

            elif event == cv2.EVENT_LBUTTONUP:
                dragging = False
                if roi is not None:
                    x0, y0, x1, y1 = roi
                    x0, y0, x1, y1 = normalize_rect(x0, y0, x1, y1)

                    # Clamp to display bounds
                    x0 = clamp(x0, 0, DISP_W - 1)
                    x1 = clamp(x1, 0, DISP_W - 1)
                    y0 = clamp(y0, 0, DISP_H - 1)
                    y1 = clamp(y1, 0, DISP_H - 1)

                    # Require non-trivial ROI
                    if (x1 - x0) < 5 or (y1 - y0) < 5:
                        roi = None
                    else:
                        roi = (x0, y0, x1, y1)

        cv2.namedWindow(window_name)
        cv2.setMouseCallback(window_name, on_mouse)

        # Get a first thermal frame for dimensions
        therm_img = self.get_cam_latest('thermal')
        therm_h, therm_w = therm_img.shape[:2]

        try:
            while True:
                rgb_img = self.get_cam_latest('color')

                if rgb_img is None or therm_img is None:
                    continue

                if transformed_view:
                    rgb_img = self.get_transformed_view(rgb_img, cam_type="color")

                # Display image is always resized to 1280x720 for consistent mouse mapping
                disp = cv2.resize(rgb_img, (DISP_W, DISP_H), interpolation=cv2.INTER_NEAREST)
                disp, finite, vmin, vmax = self.heat_overlay(disp, roi, transformed_view = transformed_view)
                    
                if np.any(finite):
                    cv2.putText(
                        disp,
                        f"ROI Temp: {vmin:.2f} .. {vmax:.2f} C",
                        (10, 20),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.55,
                        (0, 255, 0),
                        1,
                        cv2.LINE_AA,
                    )

                # Help text
                cv2.putText(
                    disp,
                    "Drag LMB to select ROI | r reset | q/ESC quit",
                    (10, DISP_H - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (0, 255, 0),
                    1,
                    cv2.LINE_AA,
                )

                cv2.imshow(window_name, disp)

                key = cv2.waitKey(1) & 0xFF
                if key == ord('r'):
                    roi = None
                elif key == ord('q') or key == 27:
                    break

        finally:
            cv2.destroyWindow(window_name)
    
    
    
if __name__ == '__main__':
    ##################################################################################
    #-------------------------------- Laser Config ----------------------------------#
    ##################################################################################
    
    laser_controller = Laser_Arduino()  # controls whether laser is on or off
    laser_on = False
    laser_controller.set_output(laser_on)
    
    ##################################################################################
    #------------------------------ Robot Config ------------------------------------#
    ##################################################################################
    # Create FrankaNode object for controlling robot
    robot_controller = Robot_Controller(laser_controller)
    # TODO fix running in container
    home_pose = robot_controller.load_home_pose()
    # home_pose = robot_controller.load_edit_pose()
    start_pos = np.array([0,0,0.10]) # [m,m,m]
    target_pose = np.array([[1.0, 0, 0, start_pos[0]],
                            [0,1,0,start_pos[1]],
                            [0,0,1,start_pos[2]],
                            [0,0,0,1]])
    robot_controller.go_to_pose(target_pose@home_pose,1) # Send robot to start position
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
    therm_cam = ThermalCam(IRFormat="TemperatureLinear10mK", height=int(480/window_scale),frame_rate="Rate50Hz",focal_distance=0.2) 
    
    ##################################################################################
    #------------------------------ RGBD Cam Config ---------------------------------#
    ##################################################################################
    rgbd_cam = RGBD_Cam() #Runs a thread internally

    
   
    
    camera_reg = Camera_Registration(therm_cam, rgbd_cam, robot_controller, laser_controller)
    # camera_reg.run()
    rgbd_cam.set_default_setting()
    # camera_reg.view_rgbd_therm_registration()
    # camera_reg.transformed_view(cam_type="thermal")
    camera_reg.live_control_view('color', warped=True, tracking=True)
    # camera_reg.view_rgbd_therm_heat_overlay()
    # camera_reg.draw_traj()
    therm_cam.deinitialize_cam()
    # camera_reg.live_control_view("color")
    # print("here")

