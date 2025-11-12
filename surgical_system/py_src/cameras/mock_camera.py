import numpy as np

class MockCamera:
    def __init__(self):
        self.latest_image = np.random.randint(0, 256, (360, 640, 3), dtype=np.uint8)
    
    def start_stream(self):
        print("Starting mock camera stream")

    def get_latest(self):
        return {'image': self.latest_image}