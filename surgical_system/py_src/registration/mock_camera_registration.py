import numpy as np
from cameras.mock_camera import MockCamera
from robot.mock_robot_controller import MockRobotController

class MockCameraRegistration():
    def __init__(self, therm_cam: MockCamera, rgbd_cam: MockCamera, robot_controller: MockRobotController, laser_obj):
        self.therm_cam = therm_cam
        self.rgbd_cam = rgbd_cam
        self.robot_controller = robot_controller
        self.laser_obj = laser_obj

    def pixel_to_world(self, img_points, cam_type, z = 0.0):
        if(isinstance(img_points, np.ndarray) and len(img_points.shape) > 1):
            print("Path processing")
            return [[0,0,z], [1,1,z]]
        else:
            return np.array([img_points[0], img_points[1], z])
    
    def get_cam_latest(self, cam_type):
        if(cam_type == "thermal"):
            return self.therm_cam.get_latest()
        else:
            return self.rgbd_cam.get_latest()