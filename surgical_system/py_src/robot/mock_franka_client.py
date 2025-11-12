import numpy as np

class MockRobotController():
    def __init__(self):
        self.pose = np.identity(4)
        pass
        
    def go_to_pose(self, pose, linTol = .05):
        print("Going to pose")
        print(pose)
        self.pose = pose
    
    def get_current_pose(self):
        print("Returning mock pose")
        return self.pose
    
    def load_home_pose(self):
        return self.get_current_pose()
