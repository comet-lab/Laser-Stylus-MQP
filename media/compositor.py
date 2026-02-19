import cv2
import numpy as np
import time
import sys
import os
from async_reciever import ReceiverDaemon

print("MEDIA BOOTING", file=sys.stderr, flush=True)

output_pipe = open('/dev/stdout', 'wb')

w, h, c = 1280, 720, 3 # rgb
# TODO env /docker global video size
# TODO env /docker global framerate
cv_shape = (w, h)
np_shape = (h, w, c)
frame_size = w * h * c
image_type = np.dtype(os.getenv("IMAGE_DTYPE", "uint8"))
homography_type = np.dtype(os.getenv("HONOGRAPHY_DTYPE", "float32"))

overlay_receiver = ReceiverDaemon(
    port = int(os.getenv("OVERLAY_PORT", 5000)),
    object_size=image_type.itemsize,
    object_count=frame_size,
    init_value=np.zeros(np_shape, np.uint8)
)

homography_receiver = ReceiverDaemon(
    port = int(os.getenv("HOMOGRAPHY_PORT", 5001)),
    object_size=homography_type.itemsize,
    object_count=9,
    init_value=np.eye(3)
)

camera_input = (np.random.rand(h//10,w//10,c) * 255).astype(np.uint8)
camera_input = cv2.resize(camera_input, (w, h), cv2.INTER_LINEAR)

target_fps = 30
frame_time = 1.0 / target_fps

def get_camera_input():
    return camera_input.copy()

def blend(base: np.ndarray, overlay: np.ndarray):
    mask = overlay.sum(axis=2) != 0
    base[mask] = overlay[mask]
    return base

overlay_receiver.start(
    lambda buffer: np.frombuffer(buffer, image_type).reshape(np_shape)
)

homography_receiver.start(
    lambda buffer: np.frombuffer(buffer, homography_type).reshape((3,3))
)

while True:
    t = time.time()
    frame = blend(get_camera_input(), overlay_receiver.read())
    warped = cv2.warpPerspective(frame, homography_receiver.read(), (w, h))
    output_pipe.write(warped.tobytes())
    output_pipe.flush()
    elapsed = time.time() - t
    sleep_time = frame_time - elapsed
    if sleep_time > 0:
        time.sleep(sleep_time)