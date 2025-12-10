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
        self.calibration_folder = "/calibration_info"
        self.rgb_cali_folder = "/calibration_info/rgb_cali/"
        self.therm_cali_folder = "/calibration_info/thermal_cali/"
        self.therm_cam.start_stream()
        self.rgbd_cam.start_stream()
        
        while not self.therm_cam.get_latest() or not self.rgbd_cam.get_latest():
            print("Waiting for camera response...")
            time.sleep(0.5)
            
        self.rgb_M = self.rgbd_cali.load_homography(fileLocation = self.rgb_cali_folder)
        self.therm_M = self.therm_cali.load_homography(fileLocation = self.therm_cali_folder)
        
    def run(self): 
        debug = True
       
        self.therm_cam.start_stream()
        self.rgbd_cam.start_stream()
        
        while not self.therm_cam.get_latest() or not self.rgbd_cam.get_latest():
            print("Waiting for camera response...")
            time.sleep(0.5)
        
        # pathToCWD = os.getcwd()        
        
        # Create checkerboard
        # self.robot_controller.load_edit_pose()
        
        # self.create_checkerboard(gridShape = np.array([9, 8]), \
        #                          saveLocation=self.calibration_folder, debug=debug)
        
        # self.laser_controller.set_output(False)
        
        # self.rgb_M = self.rgbd_cali.load_homography(fileLocation = self.rgb_cali_folder, debug = debug)
        # self.therm_M = self.therm_cali.load_homography(fileLocation = self.therm_cali_folder, debug = debug)
        
        # self.reprojection_test('color', self.rgb_M, gridShape = np.array([2, 2]), laserDuration = .15, \
        #                 debug=debug, height=0)
        
        # self.reprojection_test('thermal', self.therm_M, gridShape = np.array([2, 2]), laserDuration = .15, \
        #                 debug=debug, height=0)
        
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
            self.laser_controller.set_output(False)
            
            
            
            ### Get images ##
            therm_img = self.therm_cam.get_latest()['thermal']
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
    
    def transformed_view(self, cam_type = "color"):
        # Size of the top-down (bird's-eye) image (e.g., from calibration)
        WINDOW_NAME = "w0w"
        img = self.get_cam_latest(cam_type)
        H = self.rgb_M.copy()  # your original homography

        H_shifted, (out_w, out_h) = self.make_positive_homography(H, img.shape)

        warped = cv2.warpPerspective(img, H_shifted, (out_w, out_h))

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

    def live_control_view(self, cam_type, max_vel = 0.05, window_name="Camera", frame_key="color"):
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

                if last_point is not None:
                    cv2.circle(disp, last_point, 5, (0, 255, 0), 2)
                    
                    if cam_type == "thermal" or cam_type == "color":
                        target_position = self.pixel_to_world(last_point, cam_type)[0]
                    else:
                        raise(f"Wrong camera type: {cam_type}")
                
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
                    # print("stop")

                
                cv2.imshow(window_name, disp)

                # Press 'q' or ESC to quit
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == 27:
                    break

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
    
    


if __name__ == '__main__':
    ##################################################################################
    #------------------------------ Robot Config ------------------------------------#
    ##################################################################################
    # Create FrankaNode object for controlling robot
    robot_controller = Robot_Controller()
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

    
    ##################################################################################
    #-------------------------------- Laser Config ----------------------------------#
    ##################################################################################
    
    laser_controller = Laser_Arduino()  # controls whether laser is on or off
    laser_on = False
    laser_controller.set_output(laser_on)
    
    camera_reg = Camera_Registration(therm_cam, rgbd_cam, robot_controller, laser_controller)
    # camera_reg.run()
    rgbd_cam.set_default_setting()
    camera_reg.transformed_view()
    # camera_reg.draw_traj()
    therm_cam.deinitialize_cam()
    # camera_reg.live_control_view("color")
    # print("here")

