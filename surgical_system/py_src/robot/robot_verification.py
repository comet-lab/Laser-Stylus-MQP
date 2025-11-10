from scipy.spatial.transform import Rotation
import numpy as np

if __name__=='__main__':
    from robot_controller import Robot_Controller
else:
    from .robot_controller import Robot_Controller


def main():
    robot_controller = Robot_Controller()
    
    height = 25  # [cm] height of the laser above the surface
    numPasses = 1
    numScans = 1
    targetPos = [[0, -0.75, height], # x, y, z [cm]
                    [0, 0.75, height]]
    
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