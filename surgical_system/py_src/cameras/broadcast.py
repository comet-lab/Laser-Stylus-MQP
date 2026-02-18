import cv2
import subprocess
import numpy as np
import threading

class Broadcast:
    def __init__(self, mocking):
        self.connected = False
        self.process = None
        host = "media" if mocking else "localhost"
        rtsp_url=f"rtsp://{host}:8554/overlay"

        # ffmpeg -re -f lavfi -i testsrc=size=640x360:rate=30 -c:v libx264 -preset ultrafast -tune zerolatency -movflags +global_header -rtsp_transport tcp -f rtsp rtsp://localhost:8554/mystream

        self.ffmpeg_command = [
            'ffmpeg',
            '-f', 'rawvideo',
            '-pix_fmt', 'bgr24',
            '-s', '1280x720',
            '-framerate', '3',
            '-i', '-',
            '-c:v', 'rawvideo',
            '-f', 'rawvideo',
            'tcp://media:5000'
        ]

    @staticmethod
    def read_ffmpeg_stderr(proc):
        for line in proc.stderr:
            print("FFmpeg:", line.decode(), end='')
    
    def connect(self) -> bool:
        self.process = subprocess.Popen(self.ffmpeg_command, stdin=subprocess.PIPE, bufsize=0, stderr=subprocess.PIPE, stdout=subprocess.PIPE, text=False)
        threading.Thread(target=Broadcast.read_ffmpeg_stderr, args=(self.process,), daemon=True).start()
        self.connected = True
        return self.connected
    
    def publish_frame(self, frame):
        if not self.connected or self.process == None:
            print("not connected")
            return
        # Write the raw frame bytes to the ffmpeg process's stdin
        frame = np.ascontiguousarray(frame)
        self.process.stdin.write(frame.tobytes())
        self.process.stdin.flush()
    
    def __del__(self):
        self.process.stdin.close()
        self.process.wait()
        print("FFmpeg process closed.")