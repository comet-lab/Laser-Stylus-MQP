import os
if __name__=='__main__':
    from UDP.UDPClient import UDPClient
else:
    from .UDP.UDPClient import UDPClient
import numpy as np
import struct

class FrankaClient:
    __instance = None
    class __impl:
        def __init__(self):
            self.client = UDPClient(UDP_IP='127.0.0.1',UDP_PORT=5005)
            self.last_sent = struct.pack('@ddddddddddddb',1,0,0,0,1,0,0,0,1,0.3,0,0.4,0)
            self.last_received = []
            pass
        
        def send_pose(self, transform, mode=1):
            if np.shape(transform) != (4,4):
                raise Exception("FrankaClient: Transform should be a 4x4 transformation matrix")
            
            # We might need to change b to i so it can be an integer !!!!!
            message = struct.pack('@ddddddddddddb',transform[0,0],transform[0,1],transform[0,2],\
            transform[1,0],transform[1,1],transform[1,2],transform[2,0],transform[2,1],transform[2,2],\
            transform[0,3],transform[1,3],transform[2,3], mode)
            self.last_sent = message
            # The data is rotation matrix first row, second row, third row, translation column
            # The last character is a 0 for no control, 1 for pose control.
            # The return message should be, translation vector [x,y,z], quaternion of orientation [x,y,z,w]. (7 doubles)
            return_message = self.client.requestMessage(message)  # send return information
            self.last_received = struct.unpack_from('@ddddddd', return_message[0])
            return self.last_received
        
        def request_pose(self):
            # The return message should be, translation vector [x,y,z], quaternion of orientation [x,y,z,w]. (7 doubles)
            return_message = self.client.requestMessage(self.last_sent)  # send return information
            self.last_received = struct.unpack_from('@ddddddd', return_message[0])
            return self.last_received
        
        def stop_control(self):
            message = struct.pack('@ddddddddddddb',1,0,0,0,1,0,0,0,1,0.3,0,0.4,0)
            return_message = self.client.requestMessage(message)
            self.last_received = struct.unpack_from('@ddddddd', return_message[0])
            return self.last_received
        
        def close(self):
            self.stop_control()
            init_message = struct.pack('@ddddddddddddb',1,0,0,0,1,0,0,0,1,0.3,0,0.4,-1)
            self.client.requestMessage(init_message)



    def __init__(self):
        """ Create singleton instance """
        # Check whether we already have an instance
        if FrankaClient.__instance is None:
            # Create and remember instance
            FrankaClient.__instance = FrankaClient.__impl()
        else:
            print("FrankaClient Object Already Exists.\nReturning existing object")
        # Store instance reference as the only member in the handle
        self.__dict__['_Singleton__instance'] = FrankaClient.__instance

    def __getattr__(self, attr):
        """ Delegate access to implementation """
        return getattr(self.__instance, attr)

    def __setattr__(self, attr, value):
        """ Delegate access to implementation """
        return setattr(self.__instance, attr, value)
  
franka_client = FrankaClient()
    
if __name__=='__main__':
    import time, subprocess
    from scipy.spatial.transform import Rotation
    time.sleep(2)
    pathToCWD = os.getcwd()
    subprocess.Popen([pathToCWD + "/surgical_system/cpp_src/main"]) 
    # Load home pose 
    rot = Rotation.from_euler('ZYX',[0,np.pi/4,np.pi/2])
    rotM = rot.as_matrix()
    
    # Default robot starting location 
    home_position = np.array([[0.4425],[0.1043],[0.1985]])
    home_pose = np.concatenate((rotM,home_position),axis=1)
    home_pose = np.concatenate((home_pose,[[0,0,0,1]]),axis=0)
    
    mode = 1
    height = 0.2 # m
    x = 0
    y = 0
    target_pose = np.array([[1,0,0,x],[0,1,0,y],[0,0,1,height],[0,0,0,1]])

    time.sleep(2)
    returnedPose = franka_client.send_pose(target_pose@home_pose,mode)
    time.sleep(4)

    message = franka_client.request_pose()    
    print("Robot Message: ", message)
    franka_client.close()