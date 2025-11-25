import numpy as np
from cameras.mock_camera import MockCamera

class MockCameraRegistration():
    def __init__(self, therm_cam, rgbd_cam: MockCamera, robot_controller: MockCamera, laser_obj):
        self.therm_cam = therm_cam
        self.rgbd_cam = rgbd_cam
        self.robot_controller = robot_controller
        self.laser_obj = laser_obj

    def pixel_to_world(self, img_points, cam_type, z = 0.0):
        return np.array([img_points[0], img_points[1], z])
    
    def get_cam_latest(self, cam_type):
        return self.rgbd_cam.get_latest()
        