import cv2
import subprocess
import numpy as np
import threading

class Broadcast:
    def __init__(self, mocking):
        self.connected = False
        self.process = None
        host = "media" if mocking else "localhost"
        rtsp_url=f"rtsp://{host}:8554/mystream"

        # ffmpeg -re -f lavfi -i testsrc=size=640x360:rate=30 -c:v libx264 -preset ultrafast -tune zerolatency -movflags +global_header -rtsp_transport tcp -f rtsp rtsp://localhost:8554/mystream

        self.ffmpeg_command = [
            'ffmpeg',
            '-y',  # Overwrite output files without asking
            '-f', 'image2pipe',  # Input format is raw video
            '-vcodec', 'mjpeg',
            '-framerate', '60',
            '-i', '-',  # Read input from stdin pipe
            '-c:v', 'libx264',  # Video codec H.264
            '-preset', 'ultrafast',
            '-tune', 'zerolatency',
            '-rtsp_transport', 'tcp',
            '-f', 'rtsp',  # Output format is RTSP
            rtsp_url
        ]

    @staticmethod
    def read_ffmpeg_stderr(proc):
        for line in proc.stderr:
            print("FFmpeg:", line.decode(), end='')
    
    def connect(self) -> bool:
        self.process = subprocess.Popen(self.ffmpeg_command, stdin=subprocess.PIPE, stderr=subprocess.PIPE, stdout=subprocess.PIPE, text=False)
        threading.Thread(target=Broadcast.read_ffmpeg_stderr, args=(self.process,), daemon=True).start()
        self.connected = True
        return self.connected
    
    def publish_frame(self, frame):
        if not self.connected or self.process == None:
            print("not connected")
            return
        # Write the raw frame bytes to the ffmpeg process's stdin
        success, img_encoded = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 100])
        if(success):
            self.process.stdin.write(img_encoded.tobytes())
        else:
            print('Malformed image')
    
    def __del__(self):
        self.process.stdin.close()
        self.process.wait()
        print("FFmpeg process closed.")