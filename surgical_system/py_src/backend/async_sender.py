import socket
from typing import Any, Callable

class Sender():
    def __init__(self, host: str, port: int):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        print(host, port)
        server.connect((host, port))
        self.server = server
        self.last_sent = None

    def send(self, obj: Any, obj_to_bytes: Callable, check_equality: Callable):
        if not check_equality(obj, self.last_sent):
            self.server.sendall(obj_to_bytes(obj))
            self.last_sent = obj