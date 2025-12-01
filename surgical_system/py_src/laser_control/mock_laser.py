class MockLaser():
    def __init__(self):
        self.state = None

    def set_output(self, output):
        self.state = "ON" if output else "OFF"
        print(f"LASER {self.state}")

    def get_laser_state(self):
        return self.state