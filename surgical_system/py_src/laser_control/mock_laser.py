class MockLaser():
    def __init__(self):
        self.last_state = None
        self.vf_valid_flag = False

    def set_output(self, output):
        state = "ON" if output else "OFF"
        if(self.last_state != state):
            print(f"LASER {state}")
        self.last_state = state
