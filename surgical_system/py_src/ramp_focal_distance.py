# Import statements
import numpy as np
import time
import cv2
from laser_control.laser_arduino import Laser_Arduino
from robot.robot_controller import Robot_Controller
from cameras.RGBD_cam import RGBD_Cam
from registration.cameraRegistration import Camera_Registration
import matplotlib.pyplot as plt


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
    # # "Spot" laser focus test 
    # # Create grid of 10x10 points on 50 x 50 mm block
    # gridSize = np.array([10, 10])
    # xPoints = (np.arange(gridSize[0]) - (gridSize[0] - 1) / 2) * 0.005
    # yPoints = (np.arange(gridSize[1]) - (gridSize[1] - 1) / 2) * 0.005
    # xValues, yValues = np.meshgrid(xPoints, yPoints)
    # # Calculate the trajectory 
    # numPoints = gridSize[0] * gridSize[1]
    # zValues = np.full(numPoints, 0.02) #m
    # # Each pass of laser, robot (laser) will go down 1 cm (range 0.04 to -0.04 m)
    # for i in range(gridSize[0]):
    #     for j in range(gridSize[1]):
    #         zValues[i * gridSize[1] + j] = (i * 0.01 + j * 0.001) - 0.055

    # # "Ramp" laser focus test 
    # # TODO: Select line size (50, 100, 150, and 200 mm)
    # # TODO: Edit firing range in robot_controller.py!
    # # NOTE: Allows for extra 5 mm on each side to ensure constant velocity when firing
    # # xValues = np.arange(-0.030, 0.030, 0.001)
    # # xValues = np.arange(-0.055, 0.055, 0.001)
    # # xValues = np.arange(-0.080, 0.080, 0.001)
    # # xValues = np.arange(-0.105, 0.105, 0.001)

    # # TODO: Select line height / y value (0.00, 0.01, 0.02, 0.03,and 0.04 m)
    # yValues = np.full(xValues.shape, -0.030)

    # # Heights chosen to roughly center focal point/distance
    # zValues = np.linspace(-0.090, -0.040, xValues.shape[0])
    
    
    # Testing MDF x CO2 laser interactions
    xValues = np.arange(-0.020, 0.020, 0.001)
    # TODO: Select yValues of robot
    # yValues = np.full(xValues.shape, -0.020)
    # yValues = np.full(xValues.shape, -0.015)
    # yValues = np.full(xValues.shape, -0.010)
    # yValues = np.full(xValues.shape, 0.005)
    # yValues = np.full(xValues.shape, 0.010)
    yValues = np.full(xValues.shape, 0.015)
    X, Y = np.meshgrid(xValues, yValues)
    #TODO: Select zValues (focal height) of robot
    # Height for approximate focal distance of 0.105 m to top of MDF block
    focalPointHeight = -0.0365
    Z = np.full(X.size, focalPointHeight) #m

    robot_path = np.hstack((X.reshape((-1, 1)), 
                                    Y.reshape((-1, 1)), 
                                    Z.reshape((-1, 1))))


    # Calculate the trajectory
    max_vel = 0.005 #[m/s]
    traj = camera_reg.robot_controller.create_custom_trajectory(robot_path, max_vel)

    # Send the robot to the starting position
    input(f"Press Enter to move robot to starting position.")
    start_pos = robot_path[0, :]
    start_pose = np.eye(4)
    start_pose[:3, -1] = start_pos
    print("Heading to starting location")
    camera_reg.robot_controller.go_to_pose(start_pose @ camera_reg.robot_controller.home_pose)

    return traj

def run_traj(camera_reg, traj):
    # Run the trajectory
    input(f"Press Enter to start the trajectory.")
    # TODO/FIXME: Turn laser off!!!
    laser_on = False
    camera_reg.robot_controller.run_trajectory(traj, blocking=False, laser_on=laser_on)
    while(camera_reg.robot_controller.is_trajectory_running()):
        time.sleep(5)
        print("Trajectory is running...")
    

def main():
     # "Ramp" laser focus test 
    # # TODO: Select line size (50, 100, 150, and 200 mm)
    # # xValues = np.arange(-0.030, 0.030, 0.001)
    # # xValues = np.arange(-0.055, 0.055, 0.001)
    # xValues = np.arange(-0.080, 0.080, 0.001)
    # # xValues = np.arange(-0.105, 0.105, 0.001)
    # # TODO: Select line height / y value (0.00, 0.01, 0.02, 0.03,and 0.04 m)
    # yValues = np.full(xValues.shape, 0.00)
    # # Heights chosen to roughly center focal point/distance
    # zValues = np.linspace(-0.090, -0.040, xValues.shape[0])
    # robot_path = np.hstack((xValues.reshape((-1, 1)), 
    #                                 yValues.reshape((-1, 1)), 
    #                                 zValues.reshape((-1, 1))))
    # # Debugging after line analysis for smallest width to match with height (focal height)
    # xValuesVector = xValues.reshape((-1, 1))
    # zValuesVector = zValues.reshape((-1, 1))
    # # TODO: Change the value to find the index of the x value that corresponds to the smallest width
    # indices = np.where(np.isclose(xValuesVector, 0.587e-03, atol=0.001))[0]
    # print("Indices: ", indices)
    # print("X Values at Indices: ", xValuesVector[indices])
    # print("Z Values at Indices: ", zValuesVector[indices])

    # Setup all communication with hardware 
    camera_reg = setup()

    # Calculate traj for laser focal distance test
    traj = calc_traj(camera_reg)

    # Run trajectory while firing laser to create 9 burn lines on surface
    run_traj(camera_reg, traj)

if __name__ == "__main__":
    main()