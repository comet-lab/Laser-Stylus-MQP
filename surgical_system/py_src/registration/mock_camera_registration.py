import numpy as np
import cv2
from cameras.mock_camera import MockCamera
from robot.mock_robot_controller import MockRobotController

class MockCameraRegistration():
    def __init__(self, therm_cam: MockCamera, rgbd_cam: MockCamera, robot_controller: MockRobotController, laser_obj):
        self.therm_cam = therm_cam
        self.rgbd_cam = rgbd_cam
        self.robot_controller = robot_controller
        self.laser_obj = laser_obj
        self.display_path = False

    def pixel_to_world(self, img_points, cam_type, z = 0.0):
        if(isinstance(img_points, np.ndarray) and len(img_points.shape) > 1):
            return [[img_points[0][0],img_points[0][1],z]]
        else:
            return np.array([img_points[0], img_points[1], z])
    
    def get_cam_latest(self, cam_type):
        if(cam_type == "thermal"):
            return self.therm_cam.get_latest()[cam_type]
        else:
            return self.rgbd_cam.get_latest()[cam_type]
        
    def moving_average_smooth(self, pixels, window):
        return pixels
    
    def world_to_real(self, pixels, cam_type, z = None):
        if(z is None):
            return pixels
        elif(pixels.shape == (1,2)):
            return np.append(pixels, z)
        else:
    
            print(pixels)
            print(type(pixels))
            raise Exception("Unkown pixel type")
        
    def show_path(self, img):
        return self.rgbd_cam.get_latest() 
    
    def hide_path(self):
        pass 
    
    def get_path(self, points):
        pass 
    
    def get_world_m_to_UI(self, cam_type, world_points, warped):
        return np.zeros((1280, 2))
    
    def get_UI_to_world_m(self, cam_type, pixel, warped, z):
        ret = [[point[0], point[1], z] for point in pixel]
        return ret
    
    def heat_overlay(self, rgb_img, mask = None, roi = None, invert = False, transformed_view: bool = True,
                                alpha: float = 0.45,
                                colormap: int = cv2.COLORMAP_JET):
        raise NotImplementedError("This function should not be called on while mock robot is enabled")
        return np.zeros((720, 1280, 3)), np.array([]), 0, 0
    
    def tracking_display(self, disp, cam_type = 'color', warped = True):
        raise NotImplementedError("This function should not be called on while mock robot is enabled")
        return np.zeros((720, 1280 , 3))
    
    def get_transform_matrix(self):
        return np.array([[1.2, 0.0, 0],
                  [0.2, 1.1, 0],
                  [0.0012, 0.0035, 1]])

    def get_transformed_view(self, latest, cam_type):
        H = self.get_transform_matrix()
        return cv2.warpPerspective(latest, H, (1280//8,720//8))