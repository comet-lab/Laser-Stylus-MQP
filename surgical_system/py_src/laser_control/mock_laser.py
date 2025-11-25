class MockLaser():
    def __init__(self):
        pass

    def set_output(self, output):
        state = "ON" if output else "OFF"
        print(f"LASER {state}")