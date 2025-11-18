import pyrealsense2 as rs
import numpy as np
import cv2, threading, queue, time
if __name__=='__main__':
    from camera import Camera
else:
    from .camera import Camera

class RGBD_Cam(Camera):
    def __init__(self, width = 1280, height = 720, frame_rate = 30, pix_Per_M = 7000):
        #Threading 
        super().__init__("RGBD Camera", width, height, pix_Per_M)
        try:
            # Configure depth and color streams
            self.pipeline = rs.pipeline()
            self.config = rs.config()

            # Get device product line for setting a supporting resolution
            self.pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
            self.pipeline_profile = self.config.resolve(self.pipeline_wrapper)
            self.device = self.pipeline_profile.get_device()
            self.device_name = str(self.device.get_info(rs.camera_info.product_line))
            self.sensor = self.device.sensors[0]
            print(self.device_name, " Connected")
            self.set_color_manual(exposure_us=2000, gain=16, white_balance_k=4500)

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
            
    def set_color_manual(self, exposure_us=2000, gain=16, white_balance_k=4500):
        # Turn OFF auto exposure & auto white balance first
        if self.sensor.supports(rs.option.enable_auto_exposure):
            self.sensor.set_option(rs.option.enable_auto_exposure, 0)
        if self.sensor.supports(rs.option.enable_auto_white_balance):
            self.sensor.set_option(rs.option.enable_auto_white_balance, 0)
            
        # Set manual exposure, gain, WB (clamped to valid ranges)
        if self.sensor.supports(rs.option.exposure):
            rng = self.sensor.get_option_range(rs.option.exposure)
            self.sensor.set_option(rs.option.exposure, np.clip(exposure_us, rng.min, rng.max))
        if self.sensor.supports(rs.option.gain):
            rng = self.sensor.get_option_range(rs.option.gain)
            self.sensor.set_option(rs.option.gain, np.clip(gain, rng.min, rng.max))
        if self.sensor.supports(rs.option.white_balance):
            rng = self.sensor.get_option_range(rs.option.white_balance)
            self.sensor.set_option(rs.option.white_balance, np.clip(white_balance_k, rng.min, rng.max))
    
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
    
    def get_beam_pixel(self, img, method="Centroid", thresholdScale = 0.8):
        # If color, convert to grayscale (OpenCV expects BGR; if yours is RGB it still just sums)
        if img.ndim == 3 and img.shape[-1] >= 3:
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        else:
            gray = img
        if method == "Centroid":
            peak = np.max(gray)
            threshold = thresholdScale * peak
            mask = gray >= threshold

            y, x = np.indices(gray.shape)
            maskedImg = gray * mask 
            x_center = np.sum(x * maskedImg) / np.sum(maskedImg)
            y_center = np.sum(y * maskedImg) / np.sum(maskedImg)
            
            
            # plt.imshow(maskedImg)
            # plt.show()
            center = np.array([x_center, y_center])
            
        else:
            center = np.flip(np.asarray((np.unravel_index(np.argmax(img), img.shape))))
        print("Raw Image Peak Temp Loc: ",np.flip(np.asarray((np.unravel_index(np.argmax(img), img.shape)))))
        print("Raw Image Center of centroid: ",center)
        
        return center
    
    
        

def main():
    rgbd_cam = RGBD_Cam()
    rgbd_cam.start_stream()
    
    while (not rgbd_cam.is_ready()):
        time.sleep(0.5)
        
    while(True):
            # image = self.rgbd_cam.get_latest()
            img = rgbd_cam.get_latest()
            if img is None:
                continue 
            img = img['color']
            pixel = rgbd_cam.get_beam_pixel(img)
            print(pixel)
            rounded_pixel = np.rint(pixel).astype(int)  
            cv2.circle(img, rounded_pixel, radius=5, color=(0, 255, 0), thickness=-1)
            cv2.imshow("pixel marker", img)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
    

if __name__=='__main__':
    main()
    