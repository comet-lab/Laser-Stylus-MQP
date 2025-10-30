import sys, os, time, subprocess
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))


from surgical_system.py_src.robot.robot_controller import Robot_Controller
from cameras.thermal_cam import ThermalCam
from laser_control.laser_arduino import Laser_Arduino
import numpy as np
import matplotlib.pyplot as plt
import cv2 
from scipy.spatial.transform import Rotation

def SelectROI(homePose, targetPose, cam_obj=None):
    print("\nSelecting Region of Interest")
    if cam_obj is None:
        cam_obj = ThermalCam(IRFormat="TemperatureLinear10mK", height=120)
        cam_obj.set_acquisition_mode()
    laser_obj = Laser_Arduino()
    
    print("Firing Laser in 2 seconds")
    robot_obj = FrankaClient()
    robot_obj.send_pose(targetPose@homePose,1)
    time.sleep(2)
    print("Laser On")
    laser_obj.set_output(1)
    time.sleep(0.5)
    laser_obj.set_output(0)
    print("Laser Off")
    image_list = cam_obj.acquire_and_display_images(5, display=False, debug=False)
    im = (image_list[4] - image_list[4].min())
    im = np.array(im*255.0/im.max(),dtype=np.uint8)
    bbox = cv2.selectROI('select',im)
    rowROI = [bbox[1], bbox[1]+bbox[3]]
    colROI = [bbox[0], bbox[0]+bbox[2]]
    cv2.destroyWindow('select')
    return rowROI, colROI

# def manuallyFocusCamera(homePose, cam_obj = None):
#     print("\nStart Manually Focusing the Camera\n")
#     if cam_obj is None:
#         cam_obj = ThermalCam(IRFormat="TemperatureLinear10mK", height=120)
#         cam_obj.set_acquisition_mode()
#     laser_obj = Laser_Arduino()
#     print("Firing Laser in 2 seconds")
#     robot_obj = FrankaClient()
#     targetPose = np.array([1,0,0,0],[0,1,0,0],[0,0,1,0.15],[0,0,0,1])
#     robot_obj.send_pose(targetPose@homePose,1)
#     time.sleep(2)
#     print("Laser On")
#     laser_obj.set_output(1)
#     cam_obj.acquire_and_display_images(500, display=True, debug=True)
#     laser_obj.set_output(0)
#     plt.close()
#     print("Laser Off")

def HT_Inv(homogeneousPose):
    """
    Computes the inverse of a homogeneous transformation matrix
    """
    R = homogeneousPose[0:3, 0:3]
    t = homogeneousPose[0:3, 3]
    R_inv = R.T
    t_inv = -R_inv @ t
    return np.concatenate((np.concatenate((R_inv, t_inv.reshape(3, 1)), axis=1), [[0, 0, 0, 1]]), axis=0)
    



if __name__ == '__main__':
    pathToCWD = os.getcwd()
    robot_controller = Robot_Controller()
    subprocess.Popen([pathToCWD + "/cpp_src/main"]) 
    time.sleep(3)
    homePose = robot_controller.loadHomePose()
    robot_controller.goToPose(homePose)