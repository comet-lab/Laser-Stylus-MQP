import socket
import threading
import sys
from typing import Callable

class ReceiverDaemon():
    def __init__(self, port: int, object_size: type, object_count: int, init_value = None):
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.port = port
        self.conn = None
        self.container = [init_value]
        self.lock = threading.Lock()
        self.buffer = b''
        self.size = object_size * object_count

    def start(self, extract_buffer: Callable):
        print(f"Dispatching daemon", file=sys.stderr, flush=True)
        threading.Thread(target=self._runnable, daemon=True, args=[extract_buffer]).start()

    def _runnable(self, extract_buffer: Callable):
        # TODO handle error (Disconnect, retry connect)
        while True:
            if self.conn is None:
                self._connect()
            else:
                try:
                    self._recv_chunk(extract_buffer)
                except ConnectionError as e:
                    self._conn = None

    def _connect(self):
        self.server.bind(('0.0.0.0', self.port)) # Allow all incoming connections
        self.server.listen(1)
        print("Connecting...", file=sys.stderr, flush=True)
        conn, code = self.server.accept()
        print(f"Connected with code: {code}", file=sys.stderr, flush=True)
        self.conn = conn
    
    def _recv_chunk(self, extract_buffer: Callable):
        chunk = self.conn.recv(self.size - len(self.buffer))
        if not chunk:
            raise ConnectionError("Sender DC")
        self.buffer += chunk
        if(len(self.buffer) == self.size):
            self._write(extract_buffer(self.buffer))
            self.buffer = b''

    def _write(self, target):
        self.lock.acquire()
        self.container[0] = target
        self.lock.release()

    def read(self):
        self.lock.acquire()
        target = self.container[0]
        self.lock.release()
        return target