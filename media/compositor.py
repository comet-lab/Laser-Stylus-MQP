import socket
import cv2
import numpy as np
import threading
import time
from typing import List
import sys

# TODO mock and physical mode

print("MEDIA BOOTING", file=sys.stderr, flush=True)

H = np.eye(3) # Some homography

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind(('0.0.0.0', 5000))
server.listen(1)
conn, _ = server.accept()

output_pipe = open('/dev/stdout', 'wb')

w, h, c = 1280, 720, 3 # rgb
# TODO env /docker global video size
# TODO env /docker global framerate
cv_shape = (w, h)
np_shape = (h, w, c)
frame_size = w * h * c

camera_input = (np.random.rand(h//10,w//10,c) * 255).astype(np.uint8)
camera_input = cv2.resize(camera_input, (w, h), cv2.INTER_LINEAR)

last_known_overlay = [np.zeros_like(camera_input)]
overlay_lock = threading.Lock()
target_fps = 30
frame_time = 1.0 / target_fps

def recv_overlay(last_known_overlay: List[np.ndarray], lock):
    buffer = b''
    while True:
        chunk = conn.recv(frame_size - len(buffer))
        if not chunk:
            raise ConnectionError("Sender DC")
        buffer += chunk
        if(len(buffer) == frame_size):
            overlay = np.frombuffer(buffer, np.uint8).reshape(np_shape)
            # print(overlay.shape, file=sys.stderr, flush=True)
            # print(len(overlay), file=sys.stderr, flush=True)
            lock.acquire()
            last_known_overlay[0] = overlay
            lock.release()
            buffer = b''

def get_camera_input():
    return camera_input.copy()

def blend(base: np.ndarray, overlay: np.ndarray):
    mask = overlay.sum(axis=2) != 0
    base[mask] = overlay[mask]
    return base

threading.Thread(target=recv_overlay, daemon=True, args=[last_known_overlay, overlay_lock]).start()

while True:
    t = time.time()
    overlay_lock.acquire()
    overlay = last_known_overlay[0].copy()
    overlay_lock.release()
    frame = blend(get_camera_input(), overlay)
    warped = cv2.warpPerspective(frame, H, (w, h))
    output_pipe.write(warped.tobytes())
    output_pipe.flush()
    elapsed = time.time() - t
    sleep_time = frame_time - elapsed
    if sleep_time > 0:
        time.sleep(sleep_time)