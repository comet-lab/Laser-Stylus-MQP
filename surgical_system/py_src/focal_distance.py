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

def get_spot_diameter(img):
    # Reduces noise with gaussian filter; blurs image slightly
    img_blur = cv2.GaussianBlur(img, (5, 5), 0)

    # Converts image from BGR to HSV color space
    hsv = cv2.cvtColor(img_blur, cv2.COLOR_BGR2HSV)

    # Keeps only bright pixels and everything else becomes black
    mask = cv2.inRange(hsv, (0, 0, 200), (179, 60, 255))

    # Finds all separate blobs in the binary image
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask)

    # Returns nothing if no blob is found
    if num_labels <= 1:
        return np.array([np.nan, np.nan])

    # Creates a list of blob IDs
    valid = np.arange(1, num_labels)

    # Finds blob with biggest area
    largest_blob = max(valid, key=lambda i: stats[i, cv2.CC_STAT_AREA])

    # Center coordinates
    cx, cy = centroids[largest_blob]

    # Blob area in pixels
    area = stats[largest_blob, cv2.CC_STAT_AREA]

    # Equivalent circular diameter in pixels
    diameter = 2 * np.sqrt(area / np.pi)
    return diameter


def main():
    # Setup all communication with hardware 
    camera_reg = setup()

    # Calculate the trajectory (moving downwards in z direction)
    # TODO: Starting point is negative relative to "home position" for scanning
    heights = np.arange(-0.06, 0.000, 0.001) #m
    robot_path = np.zeros((len(heights), 3))
    robot_path[:,2] = heights
    traj = camera_reg.robot_controller.create_custom_trajectory(robot_path, 0.01)

    # Send robot to starting position (first point in trajectory)
    start_pos = robot_path[0, :]
    start_pose = np.eye(4)
    start_pose[:3, -1] = start_pos
    print("Heading to starting location")
    camera_reg.robot_controller.go_to_pose(start_pose @ camera_reg.robot_controller.home_pose)

    # Loop to calculate diameters at different heights and record them
    diameters = np.zeros(len(heights))
    for i in range(robot_path.shape[0]):
        print("Heading to height: ", heights[i])
        # Send robot/laser to predetermined point
        robot_pos = robot_path[i,:]
        robot_pose = np.eye(4)
        robot_pose[:3,-1] = robot_path[i, :]
        camera_reg.robot_controller.go_to_pose(robot_pose @ camera_reg.robot_controller.home_pose)

        # Ensure robot is no longer moving before taking image (overkill)
        time.sleep(0.25)
        
        # Get RGB camera image
        color_img = camera_reg.rgbd_cam.get_latest()['color']
        # Determine diameter of laser spot
        diameter = get_spot_diameter(color_img)
        diameters[i] = diameter

    # Determine the focal distance based on smallest diameter and its corresponding height
    min_diameter_idx = np.nanargmin(diameters)
    min_diamter = diameters[min_diameter_idx]
    focal_distance = heights[min_diameter_idx]
    print(f"Focal distance is: {focal_distance} m with beam waist diameter of: {min_diamter} pixels")

    # Simple plot of findings
    laser_heights = (heights + 0.08) * 1000
    plt.scatter(laser_heights, diameters)
    plt.xlabel("Laser Height (mm)")
    plt.ylabel("Diameter (pixels)")
    plt.title("Laser Height vs. Laser Spot Diameter")
    plt.savefig("laser_height_vs_diameter.png")

if __name__ == "__main__":
    main()