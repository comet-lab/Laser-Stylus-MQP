# Import statements
import numpy as np
import time
import cv2
from laser_control.laser_arduino import Laser_Arduino
from robot.robot_controller import Robot_Controller
from cameras.RGBD_cam import RGBD_Cam
from registration.cameraRegistration import Camera_Registration



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

    # TODO: Create loop to calculate diameters are different heights and record them
    # Get RGB camera image
    color_img = camera_reg.rgbd_cam.get_latest()['color']
    # Determine diameter of laser spot
    diameter = get_spot_diameter(color_img)



if __name__ == "__main__":
    main()