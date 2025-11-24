import numpy as np
from scipy.spatial.transform import Rotation

class MockRobotController():
    def __init__(self):
        self.pose = self.load_home_pose()
        
    def go_to_pose(self, pose, linTol = .05):
        print("Going to pose")
        print(pose)
        self.pose = pose
    
    def get_current_state(self):
        return self.pose, None
    
    def load_home_pose(self):
        rot = Rotation.from_euler('ZYX',[0,np.pi/4,np.pi/2])
        rotM = rot.as_matrix()
        
        homePosition = np.array([[0.5275],[0.0893],[0.1985]])
        homePose = np.concatenate((rotM,homePosition),axis=1)
        homePose = np.concatenate((homePose,[[0,0,0,1]]),axis=0)
        return homePose
