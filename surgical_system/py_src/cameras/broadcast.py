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

        # ffmpeg -re -f lavfi -i testsrc=size=640x360:rate=30 -c:v libx264 -preset ultrafast -tune zerolatency -movflags +global_header -rtsp_transport tcp -f rtsp rtsp://localhost:8554/mystream


    @staticmethod
    def read_ffmpeg_stderr(proc):
        for line in proc.stderr:
            print("FFmpeg:", line.decode(), end='')
    
    def connect(self, frame) -> bool:
        self.process = subprocess.Popen(
            self.build_ffmpeg(frame),
            stdin=subprocess.PIPE,
            stderr=subprocess.PIPE,
            stdout=subprocess.PIPE,
            text=False
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
        if not self.connected or self.process == None:
            print("not connected... Initializing ffmpeg process")
            self.connect(self, frame)
        self.process.stdin.write(frame.tobytes())
    
    def __del__(self):
        self.process.stdin.close()
        self.process.wait()
        print("FFmpeg process closed.")

    def build_ffmpeg(self, frame):
        height, width = frame.shape[0], frame.shape[1]
        # TODO check nvenc available
        ffmpeg_command = [
            'ffmpeg',
            '-y',  # Overwrite output files without asking
            '-f', 'rawvideo',  # Input format is raw video
            '-pix_fmt', 'bgr24',
            '-s', f'{width}x{height}',
            # '-r', '24', # framerate
            #'-re',
            "-r", "50",# framerate
            '-i', '-',  # Read input from stdin pipe
            '-c:v', 'libx264',  # Video codec H.264
            '-preset', 'ultrafast',
            '-tune', 'zerolatency',
            '-x264opts', 'sync-lookahead=0:sliced-threads=1',
            # '-c:v', 'h264_nvenc',  # Video codec nvidia encode
            # '-preset', 'llhq', # Low latency high quality
            # '-tune', 'ull', # Ultra low latency
            '-rtsp_transport', 'tcp',
            '-fflags', 'nobuffer',
            '-flags', 'low_delay',
            '-probesize', '32',
            '-analyzeduration', '0',
            '-g', '30',
            '-bf', '0',
            '-b:v', '6M',
            '-f', 'rtsp',  # Output format is RTSP
            self.rtsp_url
        ]
        return ffmpeg_command
    

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