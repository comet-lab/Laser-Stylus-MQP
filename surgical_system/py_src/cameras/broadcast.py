import cv2
import subprocess
import numpy as np
import threading

class Broadcast:
    def __init__(self, mocking):
        self.connected = False
        self.process = None
        host = "media" if mocking else "localhost"
        self.rtsp_url=f"rtsp://{host}:8554/mystream"
        self.ffmpeg_command = [
            'ffmpeg',
            '-f', 'rawvideo',
            '-pix_fmt', 'bgr24',
            '-s', '1280x720',
            '-framerate', '3',
            '-i', '-',
            '-c:v', 'rawvideo',
            '-f', 'rawvideo',
            'tcp://169.254.0.3:5000'
        ]

        # ffmpeg -re -f lavfi -i testsrc=size=640x360:rate=30 -c:v libx264 -preset ultrafast -tune zerolatency -movflags +global_header -rtsp_transport tcp -f rtsp rtsp://localhost:8554/mystream


    @staticmethod
    def read_ffmpeg_stderr(proc):
        for line in proc.stderr:
            print("FFmpeg:", line.decode(), end='')
    
    def connect(self) -> bool:
        self.process = subprocess.Popen(
            self.ffmpeg_command,
            stdin=subprocess.PIPE,
            stderr=subprocess.PIPE,
            stdout=subprocess.PIPE,
            text=False,
            bufsize=0
        )
        threading.Thread(target=Broadcast.read_ffmpeg_stderr, args=(self.process,), daemon=True).start()
        self.connected = True
        return self.connected
    
    def publish_frame(self, frame):
        # force BGR24 and contiguous memory
        # if frame.shape[2] == 4:
        #     frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        # elif frame.shape[2] == 1:
        #     frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        # frame = cv2.resize(frame, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
        frame = np.ascontiguousarray(frame)
        # if not self.connected or self.process == None:
        #     print("not connected... Initializing ffmpeg process")
        #     self.build_ffmpeg(self, frame)
        self.process.stdin.write(frame.tobytes())
        self.process.stdin.flush()
    
    def __del__(self):
        self.process.stdin.close()
        self.process.wait()
        print("FFmpeg process closed.")
    

    # old_ffmpeg_command = [
    #         'ffmpeg',
    #         '-y',  # Overwrite output files without asking
    #         '-f', 'image2pipe',  # Input format is raw video
    #         '-vcodec', 'mjpeg',
    #         '-framerate', '60',
    #         '-i', '-',  # Read input from stdin pipe
    #         '-c:v', 'libx264',  # Video codec H.264
    #         '-preset', 'ultrafast',
    #         '-tune', 'zerolatency',
    #         '-rtsp_transport', 'tcp',
    #         '-fflags', 'nobuffer',
    #         '-flags', 'low_delay',
    #         '-g', '1',
    #         '-bf', '0',
    #         '-f', 'rtsp',  # Output format is RTSP
    #         rtsp_url
    #     ]