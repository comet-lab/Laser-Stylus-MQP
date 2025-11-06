import cv2
import subprocess
import numpy as np

rtsp_url="rtsp://localhost:8554/mystream"

frameSize = (640, 360)

# ffmpeg -re -f lavfi -i testsrc=size=640x360:rate=30 -c:v libx264 -preset ultrafast -tune zerolatency -movflags +global_header -rtsp_transport tcp -f rtsp rtsp://localhost:8554/mystream

ffmpeg_command = [
    'ffmpeg',
    '-y',  # Overwrite output files without asking
    '-f', 'image2pipe',  # Input format is raw video
    '-vcodec', 'mjpeg',
    '-framerate', '30',
    '-i', '-',  # Read input from stdin pipe
    '-c:v', 'libx264',  # Video codec H.264
    '-preset', 'ultrafast',
    '-tune', 'zerolatency',
    '-rtsp_transport', 'tcp',
    '-f', 'rtsp',  # Output format is RTSP
    rtsp_url
]

class Broadcast:
    def __init__(self):
        self.connected = False
        self.process = None
    
    def connect(self) -> bool:
        self.process = subprocess.Popen(ffmpeg_command, stdin=subprocess.PIPE, stderr=subprocess.PIPE, stdout=subprocess.PIPE, text=False)
        self.connected = True
        return self.connected
    
    def publish_frame(self, frame):
        if not self.connected or self.process == None:
            print("not connected")
            return
        # Write the raw frame bytes to the ffmpeg process's stdin
        _, img_encoded = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
        self.process.stdin.write(img_encoded.tobytes())
    
    def __del__(self):
        self.process.stdin.close()
        self.process.wait()
        print("FFmpeg process closed.")