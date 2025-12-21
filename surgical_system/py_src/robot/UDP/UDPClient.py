import socket
import struct
import numpy as np

class UDPClient:
    # This is a UDP server meant to facilitate communication between C++ code to control the Franka Emika arm
    # and the temperature controllers being run in Python. 
    
    def __init__(self, UDP_IP='127.0.0.1',UDP_PORT=5005):
        print("UDP target IP: %s" % UDP_IP)
        print("UDP target port: %s" % UDP_PORT)
        self.serverIP = UDP_IP
        self.serverPort = UDP_PORT
        self.sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP

    def send(self, message):
        self.sock.sendto(message, (self.serverIP, self.serverPort))

    def requestMessage(self, message, buffer_size=2024):
        self.send(message)  # send return information
        returnMessage = self.sock.recvfrom(buffer_size)
        # print(returnMessage[0])
        return returnMessage

    def close(self):
        self.sock.close()


def testStateSwap(my_socket):
    maxCount = 20
    for i in range(3):
        mode = 1
        if i == 0:
            mode = 0
        count = 0
        while count < maxCount:
            message = my_socket.requestMessage(pose,mode)
            time.sleep(1)
            count = count + 1
            currState = struct.unpack('ddddddddddddd',message[0])
            print(currState)


if __name__=='__main__':
    import time
    import subprocess
    my_socket = UDPClient();
    mainAddress = "/home/kangzhang/CMT LAB/Laser_Control/cpp_src/main"
    subprocess.Popen([mainAddress]) 
    time.sleep(2)
    alignmentOffset = 0.015
    
    
    # my_socket.send(str(np.array([1, 2, 3])), my_socket.UDP_IP, 5005)
    pose = np.array([[1/np.sqrt(2),1/np.sqrt(2),0,0.385],[0,0,-1,0.045],[-1/np.sqrt(2),1/np.sqrt(2),0,0.355],[0,0,0,1]])
    print(pose)
    mode = 1
    my_socket.requestMessage(pose,mode)
    
    time.sleep(4)

    # # scanning
    # pauseTime = 0.1
    # totalTime = 36 # seconds
    # totalDistance = 0.03 # m
    # loop = 6
    # maxCount = round(totalTime/pauseTime)
    # for j in range(loop):
    #     for i in range(maxCount):
    #         if np.mod(j,2) == 0:
    #             pose[0,3] = pose[0,3] + totalDistance/maxCount
    #         else:
    #             pose[0,3] = pose[0,3] - totalDistance/maxCount
    #         message = my_socket.requestMessage(pose,mode)
    #         time.sleep(pauseTime)


    message = my_socket.requestMessage(pose,-1)
    print("Robot Message: ", struct.unpack_from('@ddddddd', message[0]))
    my_socket.close()