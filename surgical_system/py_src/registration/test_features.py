import time, cv2
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

if __name__=='__main__':
    import sys, pathlib
    HERE = pathlib.Path(__file__).resolve().parent
    # If this file is inside a subfolder, try parent too; stop at the first match.
    for candidate in (HERE, HERE.parent, HERE.parent.parent):
        if (candidate / "robot").is_dir() and (candidate / "cameras").is_dir() and (candidate / "laser_control").is_dir():
            sys.path.insert(0, str(candidate))
            break    
    from registration.cameraRegistration import Camera_Registration
    from registration.transformations.depth_estimation import DepthEstimation
    from registration.transformations.roi_selector import ROISelector
    
import matplotlib.pyplot as plt
from robot.robot_controller import Robot_Controller
from cameras.thermal_cam import ThermalCam
from cameras.RGBD_cam import RGBD_Cam
from laser_control.laser_arduino import Laser_Arduino

### TESTING FUNCTIONS ###
def exp_depth_scan(cam_reg, gridShape = np.array([15, 15]), squareSize = 0.002):
    input(f"Press Enter to continue depth estimation creation.")
    
    xPoints = (np.arange(gridShape[1]) - (gridShape[1] - 1) / 2) * squareSize
    yPoints = (np.arange(gridShape[0]) - (gridShape[0] - 1) / 2) * squareSize
    xValues, yValues = np.meshgrid(xPoints, yPoints)
    
    
    robot_path = np.hstack((xValues.reshape((-1, 1)), 
                            yValues.reshape((-1, 1)), 
                            np.full((xValues.size, 1), 0)))
    
    start_pos = robot_path[0, :]
    start_pose = np.eye(4)
    start_pose[:3, -1] = start_pos
    print("Heading to starting location")
    cam_reg.robot_controller.go_to_pose(start_pose @ cam_reg.robot_controller.home_pose)
    
    traj = cam_reg.robot_controller.create_custom_trajectory(robot_path, 0.02)
    depth, meta = cam_reg.scan_region_for_depth(traj)
    
    save_location = cam_reg.directory + cam_reg.calibration_folder + "/depth_map.npz"
    DepthEstimation.save_depth_npz(save_location, depth, meta)
    return depth, meta
            
    
def transformed_view(cam_reg, cam_type = "color"):
    # Size of the top-down (bird's-eye) image (e.g., from calibration)
    WINDOW_NAME = "w0w"
    img = cam_reg.get_cam_latest(cam_type)

    warped = cam_reg.cam_transforms[cam_type].warp_image_for_display(img)
    

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
    
def live_control_view(cam_reg, cam_type, max_vel = 0.05, window_name="Camera", frame_key="color", 
                        warped = True, tracking = True, depth_path = "homography_stack.npz"):
    """
    Show a live view from cam_obj and allow the user to click to get pixel locations.
    """
    input("Press Enter to continue to live control")
    cam_reg.robot_controller.go_to_pose(cam_reg.robot_controller.home_pose)
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
    
    path = "surgical_system/py_src/registration/calibration_info/"
    stack_path = path + depth_path
    print("[Depth Estimation] Loading file: ", stack_path)
    homography_stack = DepthEstimation.load_homography_stack_npz(stack_path)
    dense_stack = DepthEstimation.create_dense_stack(homography_stack, dz=0.00025)
    
    
    
    try:
        while True:
            
            frame_data = cam_reg.get_cam_latest(cam_type)

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
                disp = cam_reg.get_transformed_view(disp)
                disp = cv2.resize(disp, (1280, 720), interpolation=cv2.INTER_NEAREST)
                
            

            if last_point is not None:
                cv2.circle(disp, last_point, 5, (0, 255, 0), 2)
                target_position = cam_reg.get_UI_to_world_m(cam_type, last_point, warped, z = working_height)[0]

            
            if dragging and last_point is not None:
                target_pose = np.eye(4)
                target_pose[:3, -1] = target_position
                target_vel = cam_reg.robot_controller.live_control(target_pose, max_vel)
                cam_reg.robot_controller.set_velocity(target_vel, np.zeros(3))
                cam_reg.laser_controller.set_output(1)
            else:
                current_pose, current_vel = cam_reg.robot_controller.get_current_state()
                cam_reg.laser_controller.set_output(0)
                # print(np.linalg.norm(current_vel[:3]))
                if np.linalg.norm(current_vel[:3]) > 2e-5:
                    cam_reg.robot_controller.set_velocity(np.zeros(3), np.zeros(3))
                else:
                    robot_controller.go_to_pose(current_pose, blocking=False)
                    
            if tracking:
                curr_position = cam_reg.robot_controller.current_robot_to_world_position()
                
                u_obs, v_obs = map(float, cam_reg.get_hot_pixel(frame, method="Centroid")[:2])
                X_cmd, Y_cmd = map(float, curr_position[:2])
                z_best, err_best, uv_best, conf = DepthEstimation.estimate_depth_from_dense_stack(
                    dense_stack,
                    (u_obs, v_obs),
                    (X_cmd, Y_cmd),
                    refine=True) 
                cv2.circle(disp, np.asarray((u_obs, v_obs), dtype=np.int16), 5, (255, 255, 0), 2)
                
                # print("Actual Height [mm]: ", 6.16 - 2.23)
                # print("Predicted Z [mm]: ", z_best * 1000, "| Error: ", err_best, "| Best Prediction: ", uv_best, "| Confidence: ", conf)
    
                current_pixel_location = cam_reg.get_world_m_to_UI(cam_type, curr_position, warped)[0]
                current_pixel_location = np.asarray(current_pixel_location, dtype=np.int16)
                beam_waist = cam_reg.laser_controller.get_beam_width(curr_position[-1]) 

                laser_points = cam_reg.circle_perimeter_pixels(curr_position[:2], beam_waist/2.0)
                laser_pixels = cam_reg.get_world_m_to_UI(cam_type, laser_points, warped).astype(np.int16)

                xs = laser_pixels[:, 0]
                ys = laser_pixels[:, 1]
                h, w = disp.shape[:2]
                valid = (xs >= 0) & (xs < w) & (ys >= 0) & (ys < h)
                disp[ys[valid], xs[valid]] = (0, 255, 0)
                
                cv2.circle(disp, current_pixel_location, 5, (255, 0, 0), 2)

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
            
            cv2.putText(disp, f"Estimated Height [mm] {z_best * 1000:0.2}",
                        (x0, y0 + 4*line_h),
                        font, scale, color, thickness, cv2.LINE_AA)
                
            cv2.imshow(window_name, disp)

    finally:
        cv2.destroyWindow(window_name)
        cam_reg.laser_controller.set_output(0)

def draw_traj(cam_reg, cam_type = 'color'):
    img = cam_reg.get_cam_latest(cam_type)
    pixels = cam_reg.draw_img(img)
    pixels = cam_reg.moving_average_smooth(pixels, window=5)
    robot_path = cam_reg.pixel_to_world(pixels, cam_type)
    plt.plot(robot_path[:, 0], robot_path[:, 1])
    plt.show()
    traj = cam_reg.robot_controller.create_custom_trajectory(robot_path, 0.005)
    cam_reg.robot_controller.run_trajectory(traj)
    # print(pixels)

def rgb_thermal_data(cam_reg, rgb_img, therm_img, pixel_point, transformed_view):
    disp = rgb_img.copy()
    disp = cv2.resize(disp, (1280, 720), interpolation=cv2.INTER_NEAREST)
    therm_h, therm_w = therm_img.shape[:2]
    temp = None 
    
    if pixel_point is not None:
        if transformed_view == True:
            mouse_loc = np.array([pixel_point], dtype=np.float32)
            world_loc = cam_reg.world_to_real(mouse_loc, "color", pix_Per_M = 1)[0, :2]
            pixel_loc = cv2.perspectiveTransform(np.array([[world_loc]]), 
                                                    cam_reg.world_therm_M).reshape((2)).astype('int32')
        else:
            pixel_loc = cam_reg.rgbd_therm_cali.pixel_to_world(pixel_point)[0][:2].astype('int32')
        
        # print(f"Pixel Location {world_loc}")
        if 0 <= pixel_loc[0] < therm_w and 0 <= pixel_loc[1] < therm_h:
            temp = therm_img[pixel_loc[1], pixel_loc[0]]
    
    return temp

def view_rgbd_therm_registration(cam_reg, transformed_view = True):
    debug = True
    
    cam_reg.therm_cam.start_stream()
    cam_reg.rgbd_cam.start_stream()
    
    while not cam_reg.therm_cam.get_latest() or not cam_reg.rgbd_cam.get_latest():
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
    
    therm_img = cam_reg.get_cam_latest('thermal')
    therm_h, therm_w = therm_img.shape[:2]
    
    try:
        while True:
            
            rgb_img = cam_reg.get_cam_latest('color')
            therm_img = cam_reg.get_cam_latest('thermal')
            
            if rgb_img is None or therm_img is None:
                continue
            
            
            if transformed_view == True:
                rgb_img = cam_reg.get_transformed_view(rgb_img, cam_type="color")


            disp = rgb_img.copy()
            disp = cv2.resize(disp, (1280, 720), interpolation=cv2.INTER_NEAREST)
            
            if mouse_pos is not None:
                pixel_loc = cam_reg.get_UI_to_thermal(mouse_pos, transformed_view)[0]
                
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
        cam_reg.laser_controller.set_output(0)
    pass 

def view_rgbd_therm_heat_overlay(cam_reg, transformed_view: bool = True,
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

    cam_reg.therm_cam.start_stream()
    cam_reg.rgbd_cam.start_stream()

    while not cam_reg.therm_cam.get_latest() or not cam_reg.rgbd_cam.get_latest():
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
    therm_img = cam_reg.get_cam_latest('thermal')
    therm_h, therm_w = therm_img.shape[:2]

    try:
        while True:
            rgb_img = cam_reg.get_cam_latest('color')

            if rgb_img is None or therm_img is None:
                continue

            if transformed_view:
                rgb_img = cam_reg.get_transformed_view(rgb_img, cam_type="color")

            # Display image is always resized to 1280x720 for consistent mouse mapping
            disp = cv2.resize(rgb_img, (DISP_W, DISP_H), interpolation=cv2.INTER_NEAREST)
            disp, finite, vmin, vmax = cam_reg.heat_overlay(disp, roi=roi, transformed_view = transformed_view)
                
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
    laser_controller.vf_valid_flag = True
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
    
    # base 2.23 
    # heights = np.array([2.08, 2.65, 3.15, 3.65, 4.18, 4.73, 5.15, 5.62, 6.15, 6.64,
    #                     7.26, 7.77, 8.11, 8.67, 9.24, 9.77, 10.14]) # mm 
    
    heights = np.array([2.08, 3.05, 4.08, 5.09, 6.04, 7.06, 8.04, 9.05, 10.05]) # mm 

    
    heights = heights / 1000.0 # mm
    depth_path = "homography_stack.npz"
    # stack = camera_reg.rgb_multi_layer_scan(heights, file_name= depth_path)
    # stack = 
    # print(stack)
    # rgbd_cam.set_default_setting()
    # robot_controller.load_edit_pose()
    # camera_reg.view_rgbd_therm_registration()
    # camera_reg.transformed_view(cam_type="thermal")
    live_control_view(camera_reg, 'color', warped=True, tracking=True, depth_path= depth_path) 
    
    # camera_reg.exp_depth_scan(camera_reg, gridShape = np.array([25, 25]), squareSize = 0.035/24)
    path = "surgical_system/py_src/registration/calibration_info/depth_map.npz"
    depth, meta = DepthEstimation.load_depth_npz(path)
    DepthEstimation.plot_depth_surface(depth, meta)
    
    depth_map = DepthEstimation.patch_depth(depth)
    DepthEstimation.plot_depth_surface(depth_map, meta)
    
    # camera_reg.view_rgbd_therm_heat_overlay()
    # camera_reg.draw_traj()
    therm_cam.deinitialize_cam()
    # camera_reg.live_control_view("color")
    # print("here")

