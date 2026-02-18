import numpy as np

class MockCamera:
    def __init__(self, cam_type):
        pass
    
    def start_stream(self):
        raise NotImplementedError("Cameras Deprecated")

    def get_latest(self):
        raise NotImplementedError("Cameras Deprecated")
