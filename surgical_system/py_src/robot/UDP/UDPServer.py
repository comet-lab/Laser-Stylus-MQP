import socket
import struct
import numpy as np

class UDPServer:
    # This is a UDP server meant to facilitate communication between C++ code to control the Franka Emika arm
    # and the temperature controllers being run in Python. 
    
    def __init__(self, UDP_IP='127.0.0.1',UDP_PORT=5005):
        print("UDP target IP: %s" % UDP_IP)
        print("UDP target port: %s" % UDP_PORT)
        self.UDP_IP = UDP_IP
        self.UDP_PORT = UDP_PORT
        self.sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
        self.sock.bind((self.UDP_IP, self.UDP_PORT))  # Binding is import for server side.

    def send(self, message, host, port):
        self.sock.sendto(message, (host, port))

    def waitForMessage(self, returnData, buffer_size=1024):
        message, address = self.sock.recvfrom(buffer_size)
        if np.shape(returnData) != (4,4):
            raise Exception("returnData should be a 4x4 transformation matrix")
        returnBuffer = struct.pack('@ddddddddddddi',returnData[0,0],returnData[0,1],returnData[0,2],\
            returnData[1,0],returnData[1,1],returnData[1,2],returnData[2,0],returnData[2,1],returnData[2,2],\
            returnData[0,3],returnData[1,3],returnData[2,3],0)
            # The data is rotation matrix first row, second row, third row, translation column
        self.send(returnBuffer,address[0],address[1])  # send return information
        message = struct.unpack('d'*13, message)
        currPos = message[0:3]
        currOrien = message[3:]
        
        return (currPos,currOrien)

    def close(self):
        self.sock.close()


if __name__=='__main__':
    import time

    my_socket = UDPServer()

    # my_socket.send(str(np.array([1, 2, 3])), my_socket.UDP_IP, 5005)
    pose = np.array([[1,0,0,0.3],[0,1,0,0],[0,0,1,0.5],[0,0,0,1]])
    while 1:
        message = my_socket.waitForMessage(pose)
        # time.sleep(0.5)
        print(message)
    my_socket.close()