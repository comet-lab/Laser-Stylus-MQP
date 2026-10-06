# Import statements
import numpy as np
import time
from laser_control.laser_arduino import Laser_Arduino
from robot.robot_controller import Robot_Controller
from cameras.RGBD_cam import RGBD_Cam
from registration.cameraRegistration import Camera_Registration
from registration.transformations.depth_estimation import DepthEstimation

def setup():
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
    

    # FIMXE: Remove thermal camera dependence
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
    # TODO: Uncomment me for thermal camera
    #therm_cam = ThermalCam(IRFormat="TemperatureLinear10mK", height=int(480/window_scale),frame_rate="Rate50Hz",focal_distance=0.2) 
    
    ##################################################################################
    #------------------------------ RGBD Cam Config ---------------------------------#
    ##################################################################################
    rgbd_cam = RGBD_Cam() #Runs a thread internally
    rgbd_cam.display_color_stream()
    
    ### -------------------- Run calibration ---------------- ####
    # # # Create camera registration object to perform calibration and surface scan
    camera_reg = Camera_Registration(therm_cam,rgbd_cam, robot_controller, laser_controller)

    # Define home pose of robot
    camera_reg.robot_controller.load_edit_pose()

    return camera_reg

def calc_traj(camera_reg):

    # Extract X,Y,Z coordinates of points in most recent depth reconstruction to define trajectory for raster scan
    path = "surgical_system/py_src/registration/calibration_info/depth_map.npz"
    depth, meta = DepthEstimation.load_depth_npz(path)
    depth_map = DepthEstimation.patch_depth(depth)
    Z = np.asarray(depth_map, dtype=float)
    H, W = Z.shape
    cs = float(meta["cell_size"])
    xs = float(meta["xmin"]) + (np.arange(W) + 0.5) * cs
    ys = float(meta["ymin"]) + (np.arange(H) + 0.5) * cs
    X, Y = np.meshgrid(xs, ys) 
    # Alters dimensions to have N x 3 recon_points
    recon_points = np.stack([X, Y, Z], axis=-1).reshape(-1, 3)


    # FIXME: Calculate focal distance 
    # Notes: 
        # Surface is 44 mm above ground plane 
        # Robot zero position is 198.5 mm in z
        # Focal position is approixmately 105 mm
        # "Zero" starting position of robot in focus is 198.5 - (44 + z_recon) + dist_to_focus = 105 mm 
    robot_path = recon_points
    dist_to_focus = recon_points[:,2] - 0.0495
    print(f"dist_to_focus: {dist_to_focus}")
    # TODO: Adjust z position to be at focal distance (choose if AF or NAF)
    robot_path[:,2] = dist_to_focus[0] * np.ones(len(np.array(robot_path[:,2]))) # For NAF, only choose dist_to_focus[0] index 
    

    # Calculate the trajectory (must change depending upon object being scanned)
    max_vel = 0.00467 #[m/s] - 0.005 m/s on slant, but only control horizontal velocity
    traj = camera_reg.robot_controller.create_custom_trajectory(robot_path, max_vel)

    # Send the robot to the starting position
    start_pos = robot_path[0, :]
    start_pose = np.eye(4)
    start_pose[:3, -1] = start_pos
    print("Heading to starting location")
    camera_reg.robot_controller.go_to_pose(start_pose @ camera_reg.robot_controller.home_pose)

    return traj


def run_traj(camera_reg, traj):
    # Run the trajectory
    # TODO/FIXME: Turn laser off!!!
    laser_on = True
    camera_reg.robot_controller.run_trajectory(traj, blocking=False, laser_on=laser_on)
    while(camera_reg.robot_controller.is_trajectory_running()):
        time.sleep(5)
        print("Trajectory is running...")

def main():
    # Setup all communication with hardware 
    camera_reg = setup()

    input(f"Press Enter to perform the raster scan with focus control...")

    # Calculate trajectory for raster scan with focus control
    traj = calc_traj(camera_reg)
    # Run the trajectory
    run_traj(camera_reg, traj)

if __name__ == "__main__":
    main()