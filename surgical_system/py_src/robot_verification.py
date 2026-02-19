from PIL import Image
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
import numpy as np
import cv2

from motion_planning.motion_planning import Motion_Planner
from robot.mock_robot_controller import MockRobotController
from laser_control.mock_laser import MockLaser


def main():
    img_path = "surgical_system/py_src/motion_planning/path2.png"
    img = np.array(Image.open(img_path))
    img = Motion_Planner.fill_in_shape(img)
    img = cv2.resize(img, (1280, 720), interpolation=cv2.INTER_NEAREST)
    if img.ndim == 3 and img.shape[2] == 4:
        gray = cv2.cvtColor(img, cv2.COLOR_BGRA2GRAY)
    elif img.ndim == 3:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    else:
        gray = img 
        
    polygon, edge = Motion_Planner._create_polygon(gray)
    # plot_polygon(polygon, show_vertices=True)
    waypoints = Motion_Planner.poly_raster(
        polygon,
        spacing=35.0,        # pixels
        theta_deg=45.0,      # angle
        margin=10.0          # inward offset
    )
    waypoints = Motion_Planner.smooth_corners_fillet(waypoints, radius=5, n_arc=10)
    
    # plt.plot(waypoints[:, 0], waypoints[:, 1])
    # plt.plot(edge[:, 0], edge[:, 1])
    # plt.show()
    
    laser_obj = MockLaser()
    mock_robot = MockRobotController(laser_obj)
    
    
    traj = mock_robot.create_custom_trajectory(waypoints, 3)
    positions = traj.target_position_list
    
    
if __name__=='__main__':
    main()