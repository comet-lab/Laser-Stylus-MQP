from franka_client import FrankaClient

class MockFrankaClient(FrankaClient):
    def __init__(self):
        self.__instance = self
        
    def send_pose(self, transform, mode=1):
        print("Sending mock message")
        return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    
    def request_pose(self):
        print("Returning mock pose")
        return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    def close(self):
        print("Closing mock client")
