from scipy.spatial.transform import Rotation
import numpy as np
import time

if __name__=='__main__':
    from robot.robot_controller import Robot_Controller
else:
    from .robot.robot_controller import Robot_Controller


def main():
    robot_controller = Robot_Controller()
    home_pose = robot_controller.get_home_pose()
    
    mode = 1
    height = 0.2 # m
    x = 0
    y = 0
    target_pose = np.array([[1,0,0,x],[0,1,0,y],[0,0,1,height],[0,0,0,1]])

    time.sleep(2)
    returnedPose = robot_controller.go_to_pose(target_pose@home_pose,mode)
    
    height = 25  # [cm] height of the laser above the surface
    numPasses = 1
    numScans = 1
    targetPos = np.array([[0, -1.75, height], # x, y, z [cm]
                        [0, 1.75, height]])
    
    # targetPos = [[-0.75, -0.75, height], # x, y, z [cm]
    #                 [0.75, 0.75, height]]


    gains = {"Positions": targetPos, 
             "Pattern": "Line",
             "Passes": numPasses,
             "MaxVelocity": 0,
             "MaxAcceleration": 0,
             "Durations":[10]} # accumulated time 
    
    traj = robot_controller.create_trajectory(gains)
    robot_controller.run_trajectory(traj, 1)
    

    
if __name__=='__main__':
    main()