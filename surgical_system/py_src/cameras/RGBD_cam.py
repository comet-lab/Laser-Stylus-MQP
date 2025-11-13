import pyrealsense2 as rs
import numpy as np
import cv2, threading, queue, time
if __name__=='__main__':
    from camera import Camera
else:
    from .camera import Camera

class RGBD_Cam(Camera):
    def __init__(self, width = 640, height = 480, frame_rate = 30, pix_Per_M = 7000):
        #Threading 
        super().__init__(width, height, pix_Per_M) 
        try:
            # Configure depth and color streams
            self.pipeline = rs.pipeline()
            self.config = rs.config()

            # Get device product line for setting a supporting resolution
            self.pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
            self.pipeline_profile = self.config.resolve(self.pipeline_wrapper)
            self.device = self.pipeline_profile.get_device()
            self.device_name = str(self.device.get_info(rs.camera_info.product_line))
            print(self.device_name, " Connected")

            self.config.enable_stream(rs.stream.depth, width, height, rs.format.z16, frame_rate)
            self.config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, frame_rate)
            self.pipeline.start(self.config)
            
            self.thread = threading.Thread(target=self._run, daemon=True)
            print(f"Starting {self.device_name} Thread...")
            self._ready = True
            self.thread.start()
            
        except:
            print("RGBD failed to connect or Init")
    
    def _run(self):
        try:
            if not self._ready:
                return 
            while True:
                self.thread_ready.wait()
                frames = self.pipeline.wait_for_frames()
                depth = frames.get_depth_frame()
                color = frames.get_color_frame()
                if not depth or not color:
                    continue
                depth_np = np.asanyarray(depth.get_data())
                color_np = np.asanyarray(color.get_data())
                ts = frames.get_timestamp()  # milliseconds
                item = {"color": color_np, "depth": depth_np, "ts": ts}

                self._lock.acquire()
                self._latest = item
                self._lock.release()
        finally:
            self.pipeline.stop()
            
    # def get_latest(self):
    #     """Return the most recent {'color','depth','ts'} or None if nothing yet."""
    #     self._lock.acquire()
    #     item = self._latest
    #     self._lock.release()
    #     return item
    
    # def is_ready(self):
    #     return self._ready

    # def stop_stream(self):
    #     self.thread_ready.clear()
        
    # def start_stream(self):
    #     self.thread_ready.set()
    
    def display_depth_stream(self):
        depth_image = self.get_latest()['depth']
        if depth_image is None:
            return
        depth_image = depth_image['depth']
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        # Show images
        cv2.namedWindow('Color Stream', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('Depth Stream', depth_colormap)
        cv2.waitKey(1)
        
    def display_color_stream(self):
        color_image = self.get_latest()
        if color_image is None:
            return
        color_image = color_image['color']
        cv2.namedWindow('Color Stream', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('Depth Stream', color_image)
        cv2.waitKey(1)
        
    def display_all_streams(self):
        image = self.get_latest()
        if image is None:
            return 
        color_image = image['color']
        depth_image = image['depth']
        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        # If depth and color resolutions are different, resize color image to match depth image for display
        if depth_image.shape != color_image.shape:
            resized_color_image = cv2.resize(color_image, dsize=(depth_image.shape[1], depth_image.shape[0]), interpolation=cv2.INTER_AREA)
            images = np.hstack((resized_color_image, depth_colormap))
        else:
            images = np.hstack((color_image, depth_colormap))

        # Show images
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', images)
        cv2.waitKey(1)
        

def main():
    rgbd_cam = RGBD_Cam()
    rgbd_cam.start_stream()
    
    while (not rgbd_cam.is_ready()):
        time.sleep(0.5)
        
    while(True):
        rgbd_cam.display_all_streams()
    

if __name__=='__main__':
    main()
    