class MockLaser():
    def __init__(self):
        self.state = None

    def set_output(self, output):
        self.state = "ON" if output else "OFF"

    def get_laser_state(self):
        return self.state