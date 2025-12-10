class MockLaser():
    def __init__(self):
        self.last_state = None

    def set_output(self, output):
        state = "ON" if output else "OFF"
        if(self.last_state != state):
            print(f"LASER {state}")
        self.last_state = state
