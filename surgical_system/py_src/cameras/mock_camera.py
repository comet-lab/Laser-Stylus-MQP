import numpy as np

class MockCamera:
    def __init__(self, cam_type):
        self.latest_image = np.random.randint(0, 256, (360, 640, 3), dtype=np.uint8)
        self.cam_type = cam_type
    
    def start_stream(self):
        print("Starting mock camera stream")

    def get_latest(self):
        return {self.cam_type: self.latest_image}