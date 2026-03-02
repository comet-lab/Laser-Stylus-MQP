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
    from registration.transformations.system_calibration import System_Calibration
    from registration.transformations.roi_selector import ROISelector
    
else:
    from registration.transformations.system_calibration import System_Calibration
    
from robot.robot_controller import Robot_Controller
from cameras.thermal_cam import ThermalCam
from cameras.RGBD_cam import RGBD_Cam
from laser_control.laser_arduino import Laser_Arduino
from registration.transformations.depth_estimation import DepthEstimation


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
        self.depth_path = "/homography_stack.npz"
        self.stack_path = self.directory + self.calibration_folder + self.depth_path
        print("[Depth Estimation] Loading file: ", self.stack_path)
        self.homography_stack = DepthEstimation.load_homography_stack_npz(self.stack_path)
        self.dense_stack = DepthEstimation.create_dense_stack(self.homography_stack, dz=0.00025)

        
    @staticmethod
    def circle_perimeter_pixels(center, r, num_pts=360):
            cx, cy = center
            theta = np.linspace(0, 2*np.pi, num_pts, endpoint=False)
            xs = cx + r * np.cos(theta)
            ys = cy + r * np.sin(theta)
            return np.stack([xs, ys], axis=1).astype(np.float16)
        
    def run(self): 
        debug = True
        self.homography_stack = None
       
        self.therm_cam.start_stream()
        self.rgbd_cam.start_stream()
        
        while not self.therm_cam.get_latest() or not self.rgbd_cam.get_latest():
            print("Waiting for camera response...")
            time.sleep(0.5)
        
        # pathToCWD = os.getcwd()        
        
        # Create checkerboard
        self.robot_controller.load_edit_pose()
        
        # self.create_checkerboard(gridShape = np.array([9, 8]), \
        #                          saveLocation=self.calibration_folder, debug=debug)
        
        self.create_checkerboard(gridShape = np.array([3, 3]), \
                                squareSize=0.043/2.0,
                                 saveLocation=self.calibration_folder, debug=debug)
        
        self.read_calibration()
        
        self.laser_controller.set_output(False)
        
        
        self.reprojection_test('color', self.cam_M['color'], gridShape = np.array([2, 2]), laserDuration = .15, \
                        debug=debug, height=0)
        
        self.reprojection_test('thermal', self.cam_M['thermal'], gridShape = np.array([2, 2]), laserDuration = .15, \
                        debug=debug, height=0)
        
        self.laser_alignment()
        
        heights = np.array([2.08, 2.65, 3.15, 3.65, 4.18, 4.73, 5.15, 5.62, 6.15, 6.64,
                        7.26, 7.77, 8.11, 8.67, 9.24, 9.77, 10.14]) # mm 
    
        heights = heights / 1000.0 # mm
        depth_path = "homography_stack.npz"
        camera_reg.rgb_multi_layer_scan(heights, file_name= depth_path)
        
        self.homography_stack = DepthEstimation.load_homography_stack_npz(self.stack_path)
        self.dense_stack = DepthEstimation.create_dense_stack(self.homography_stack, dz=0.00025)
    
        # self.therm_cam.deinitialize_cam()
        # pass
    
    
    def laser_alignment(self):
        # Create a copy of the original home pose which we will edit in future functions   
        new_home_pose = self.robot_controller.home_pose.copy()
        
        new_home_pose = self.xy_orientation_Correction()
    
        new_home_pose, robotError = self.findRobotOffset(new_home_pose)
        
        new_home_pose = self.robot_controller.align_robot_input(new_home_pose)
        self.robot_controller.home_pose = new_home_pose
        self.robot_controller.home_pose = new_home_pose

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
        self.robot_controller.go_to_pose(targetPose@self.robot_controller.home_pose)
        height = float(input("\nLaser-Robot Alignment: Enter max height [m]... "))
        imgCount = int(input("\nLaser-Robot Alignment: Enter num pulses... "))

        # [m] heights to test
        targetHeights = np.linspace(height, 0, imgCount)
        # initialize error
        xOrientationOffset, yOrientationOffset = 10, 10
        # make copy of homePose for edits
        _homePose = self.robot_controller.home_pose.copy()
        
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
        robot_positions = np.zeros((1, 2))

        print("\nFiring in 4 seconds ...")
        for i, (x, y) in enumerate(zip(xValues.flatten(),
                                yValues.flatten()),    
                                start=0):
            targetPose[0:3,3] = [x, y, 0]
            print(f"\nMoving to position: {targetPose[0:3,3]}")
            self.robot_controller.go_to_pose(targetPose@self.robot_controller.home_pose)
            print("Firing...")
            self.laser_controller.set_output(True)
            time.sleep(laserDuration)
            self.laser_controller.set_output(False)
            
            
            ### Get images ##
            therm_img = self.therm_cam.get_latest()['thermal']
            time.sleep(0.2) # laser may still be firing
            color_img = self.rgbd_cam.get_latest()['color']
            
            
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
    
    def rgb_multi_layer_scan(self, heights, file_name = "homography_stack.npz"):
        homography_stack = {}
        
        for z in heights:
            print("Height [mm]: ", z*1000)
            
            H = self.rgb_raster_scan(gridShape = np.array([3, 3]), \
                                     squareSize=0.035/2.0)
            homography_stack[z] = H
            
            
        try:
            file_location = os.path.join(
                "surgical_system", "py_src", "registration", "calibration_info"
            )
            os.makedirs(file_location, exist_ok=True)

            file_path = os.path.join(file_location, file_name)

            # Convert dict → arrays
            zs = np.array(sorted(homography_stack.keys()), dtype=float)
            Hs = np.stack([homography_stack[z] for z in zs], axis=0)  # (N,3,3)

            np.savez(file_path, zs=zs, Hs=Hs)

        except:
            print("Unable to write to file")
        
        return homography_stack
    
    def rgb_raster_scan(self, gridShape = np.array([2, 6]), squareSize = 0.0026):
        input(f"Press Enter to continue depth estimation creation.")
        xPoints = (np.arange(gridShape[1]) - (gridShape[1] - 1) / 2) * squareSize
        yPoints = (np.arange(gridShape[0]) - (gridShape[0] - 1) / 2) * squareSize
        xValues, yValues = np.meshgrid(xPoints, yPoints)
        robot_positions = np.zeros((0, 2))
        pixel_points = np.empty((0, 2))
        
        robot_path = np.hstack((xValues.reshape((-1, 1)), 
                                yValues.reshape((-1, 1)), 
                                np.full((xValues.size, 1), 0)))
        
        start_pos = robot_path[0, :]
        start_pose = np.eye(4)
        start_pose[:3, -1] = start_pos
        print("Heading to starting location")
        self.robot_controller.go_to_pose(start_pose @ self.robot_controller.home_pose)
        
        traj = self.robot_controller.create_custom_trajectory(robot_path, 0.025)
        self.robot_controller.run_trajectory(traj, blocking=False)
        
        # time.sleep(1)
        while(self.robot_controller.is_trajectory_running()):
            world_pos = self.robot_controller.current_robot_to_world_position()[:2]
            robot_positions = np.vstack((robot_positions, world_pos))
            color_img = self.rgbd_cam.get_latest()['color']
            rgb_laser_pixel = self.get_hot_pixel(color_img, method="Centroid")
            pixel_points = np.vstack((pixel_points, rgb_laser_pixel))
            # print("World Pos: ", world_pos, " Pixel: ", rgb_laser_pixel)
            time.sleep(1/30.0)
        # self.robot_controller.set_velocity(np.zeros(3), np.zeros(3))
            
        _imgpts = np.array(pixel_points, np.float32) # already is [2xn]
        _objpts = np.array(robot_positions, np.float32) # need this to be [2 x n] 

        # print(_objpts.shape, _imgpts.shape)
        H,_ = cv2.findHomography(_imgpts, _objpts)
        
        proj_world = DepthEstimation.project_xy(H, _imgpts)
        world_error = np.linalg.norm(_objpts - proj_world, axis=1)
        print("World reprojection error [mm]:", np.mean(world_error) * 1000)
        
        return H

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
            tuple(pts_i[3, 0]),
            BLACK,
            1, #  thickness,
            cv2.LINE_AA,  
            0, # Shift 
            0.1 # Tip Length
        )
    
        # ---- Alpha blend overlay onto original ----
        cv2.addWeighted(overlay, alpha, out, 1 - alpha, 0, out)


        return out
    
    def heat_overlay(self, rgb_img, mask = None, roi = None, invert = False, 
                                transformed_view: bool = True,
                                alpha: float = 0.45,
                                colormap: int = cv2.COLORMAP_JET):

        def normalize_rect(x0, y0, x1, y1):
            xa, xb = sorted([x0, x1])
            ya, yb = sorted([y0, y1])
            return xa, ya, xb, yb
        
        therm_img = self.get_cam_latest('thermal')
        therm_h, therm_w = therm_img.shape[:2]
        disp = rgb_img.copy()
        H, W = disp.shape[:2]
        vmin, vmax = None, None
        
        
        if mask is None and roi is None:
            return disp, None, therm_img, None
        

            
        if roi is not None:
            sel = np.zeros((H, W), dtype=bool)
            x0, y0, x1, y1 = normalize_rect(*roi)
            x0, y0 = max(0, x0), max(0, y0)
            x1, y1 = min(W - 1, x1), min(H - 1, y1)
            sel[y0:y1+1, x0:x1+1] = True

            
            # cv2.rectangle(disp, (x0, y0), (x1, y1), (0, 255, 0), 1)

        else:  # mask is not None
            
            mask_bool = ~mask.astype(bool) if invert else mask.astype(bool)
            if mask_bool.shape[:2] != (H, W):
                raise ValueError(f"mask shape {mask_bool.shape[:2]} must match image {(H, W)}")
            sel = mask_bool

        if not np.any(sel):
            return disp, sel, therm_img, None

        
        # Bounding Box
        ys, xs = np.nonzero(sel)
        y0, y1 = int(ys.min()), int(ys.max())
        x0, x1 = int(xs.min()), int(xs.max())

        # local selection mask for bbox
        sel_local = sel[y0:y1+1, x0:x1+1]
        roi_h, roi_w = sel_local.shape
        
        ly, lx = np.nonzero(sel_local)  # local coords
        pts_disp = np.stack([lx + x0, ly + y0], axis=1).astype(np.float32)  # (N,2) in (x,y)

        if transformed_view:
            world_pts = self.cam_transforms['color'].disp_px_to_world_m(pts_disp)
            therm_uv = self.cam_transforms['thermal'].world_m_to_img_px(world_pts).astype('int32')
        else:
            therm_uv = self.cam_transforms['rgb_thermal'].img_px_to_world_m(pts_disp)[:, :2].astype('int32')

        # bounds checking
        tx = np.rint(therm_uv[:, 0]).astype(np.int32)
        ty = np.rint(therm_uv[:, 1]).astype(np.int32)

        valid = (tx >= 0) & (tx < therm_w) & (ty >= 0) & (ty < therm_h)

        heat_img = np.full((roi_h, roi_w), np.nan, dtype=np.float32)

        # write only where selection exists and mapping valid
        lx_v = lx[valid]
        ly_v = ly[valid]
        heat_img[ly_v, lx_v] = therm_img[ty[valid], tx[valid]].astype(np.float32)
        
        vmin = np.nanmin(heat_img) 
        vmax = np.nanmax(heat_img)

        # --- Fixed temperature range ---
        T_MIN = 20.0
        T_MAX = 100.0   

        # Clip temperatures
        heat_clipped = np.clip(heat_img, T_MIN, T_MAX)

        # Normalize to [0,255] using FIXED range
        norm = np.zeros_like(heat_clipped, dtype=np.uint8)
        finite = np.isfinite(heat_clipped)

        if np.any(finite):
            norm[finite] = (
                255.0 * (heat_clipped[finite] - T_MIN) / (T_MAX - T_MIN)
            ).astype(np.uint8)

        # Colorize
        heat_color = cv2.applyColorMap(norm, colormap)

        # mask out invalid pixels from overlay
        if np.any(~finite):
            heat_color[~finite] = 0

        # Alpha blend overlay 
        roi_bgr = disp[y0:y1 + 1, x0:x1 + 1]
        disp[y0:y1 + 1, x0:x1 + 1] = cv2.addWeighted(heat_color, alpha, roi_bgr, 1.0 - alpha, 0)
        return disp, sel, therm_img, vmax
    
    
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
        disp[ys[valid], xs[valid]] = (0, 0, 255)
        
        cv2.circle(disp, current_pixel_location, 5, (255, 0, 0), 2)
        return disp
    
    
        
    def scan_region_for_depth(self, traj):
        self.robot_controller.run_trajectory(traj, blocking=False)
        points = np.zeros((0, 3))
        depths = []
        
        print("[Camera Reg] Scanning Features")
        while(self.robot_controller.is_trajectory_running()):
            curr_position = self.robot_controller.current_robot_to_world_position()[:2]
            color_img = self.rgbd_cam.get_latest()['color']
                    
            u_obs, v_obs = map(float, self.get_hot_pixel(color_img, method="Centroid")[:2])
            X_cmd, Y_cmd = map(float, curr_position[:2])
            z_best, err_best, uv_best, conf = DepthEstimation.estimate_depth_from_dense_stack(
                self.dense_stack,
                (u_obs, v_obs),
                (X_cmd, Y_cmd),
                refine=True) 
            
            point = np.append(curr_position, z_best)
            points = np.vstack((points, point))
            print("[Camera Reg scan_region] Points Scanned: ", point)
            # print("World Pos: ", world_pos, " Pixel: ", rgb_laser_pixel)
            time.sleep(1/30.0)
            
        depth, meta = DepthEstimation.generate_depth_mapping(points, cell_size=0.001)
        
        return depth, meta
 