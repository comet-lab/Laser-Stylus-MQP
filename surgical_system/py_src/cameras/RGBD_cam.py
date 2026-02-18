import pyrealsense2 as rs
import numpy as np
import cv2, threading, time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

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
            self.dmin = 0.06
            self.dmax = 0.1

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
            self.profile = self.pipeline.start(self.config)
            
            self.depth_sensor = self.profile.get_device().first_depth_sensor()
            self.depth_scale = self.depth_sensor.get_depth_scale()

            # self.tune_depth()
            # self.recommended_filters = self.depth_sensor.get_recommended_filters()
            self.thresh = rs.threshold_filter()
            # distances are in meters
            self.thresh.set_option(rs.option.min_distance, self.dmin)   # 6 cm
            self.thresh.set_option(rs.option.max_distance, self.dmax) 
            
            
            self.depth_to_disparity = rs.disparity_transform(True)
            self.disparity_to_depth = rs.disparity_transform(False)

            self.decimation = rs.decimation_filter()
            self.decimation.set_option(rs.option.filter_magnitude, 2)
            
            self.spatial = rs.spatial_filter()
            self.spatial.set_option(rs.option.filter_smooth_alpha, 0.4)
            self.spatial.set_option(rs.option.filter_smooth_delta, 20)
            self.spatial.set_option(rs.option.filter_magnitude, 2)
            
            self.temporal = rs.temporal_filter()
            self.temporal.set_option(rs.option.filter_smooth_alpha, 0.2)
            self.temporal.set_option(rs.option.filter_smooth_delta, 10)
            
            PERSISTENCY_MODES = {
                "disabled": 0,
                "valid_8_of_8": 1,
                "valid_2_of_3": 2,
                "valid_2_of_4": 3,   # default
                "valid_2_of_8": 4,
                "valid_1_of_2": 5,
                "valid_1_of_5": 6,
                "valid_1_of_8": 7,   
                "persist_indefinitely": 8,
            }

            self.temporal.set_option(rs.option.holes_fill, PERSISTENCY_MODES["valid_1_of_8"])

            self.hole_filling = rs.hole_filling_filter()
            self.hole_filling = rs.hole_filling_filter(2)
            
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
                frames = self.pipeline.poll_for_frames()
                if frames:
                    
                    depth = frames.get_depth_frame()
                    color = frames.get_color_frame()
                    raw = np.asanyarray(depth.get_data())
                    # print(
                    #     "scale:", self.depth_scale,
                    #     "raw mean:", raw.mean(),
                    #     "meters mean:", raw.mean() * self.depth_scale
                    # )

                    # for f in self.recommended_filters:
                    #     depth = f.process(depth)
                    if not depth or not color:
                        continue
                    
                    # depth = self.thresh.process(depth)
                    # depth = self.depth_to_disparity.process(depth)
                    # depth = self.decimation.process(depth)
                    # depth = self.spatial.process(depth)
                    # depth = self.temporal.process(depth)
                    # depth = self.hole_filling.process(depth)
                    # depth = self.disparity_to_depth.process(depth)
                    depth_np = np.asanyarray(depth.get_data()) * self.depth_scale
                    
                    color_np = np.asanyarray(color.get_data())
                    ts = frames.get_timestamp()  # milliseconds
                    item = {"color": color_np, "depth": depth_np, "ts": ts}

                    self._lock.acquire()
                    self._latest = item
                    self._lock.release()
                else:
                    time.sleep(0.0005)
        finally:
            self.pipeline.stop()
            
    def tune_depth(self, min_distance=0.07, max_distance = 0.2):
        def safe_set(opt, val):
            # 1) Check if this option is supported by the sensor
            if self.depth_sensor.supports(opt):
                # 2) Clamp value to the valid range for this option
                rng = self.depth_sensor.get_option_range(opt)
                v = float(np.clip(val, rng.min, rng.max))
                # 3) Set the option
                self.depth_sensor.set_option(opt, v)
                print(f"Set {opt.name} to {v}")
            else:
                print(f"Depth sensor does not support option {opt.name}")
                
        if self.depth_sensor.supports(rs.option.min_distance):
            safe_set(rs.option.min_distance, min_distance)
        if self.depth_sensor.supports(rs.option.max_distance):
            safe_set(rs.option.max_distance, max_distance)
    
    def set_default_setting(self):
        if self.sensor.supports(rs.option.enable_auto_exposure):
            self.sensor.set_option(rs.option.enable_auto_exposure, 1)
        if self.sensor.supports(rs.option.enable_auto_white_balance):
            self.sensor.set_option(rs.option.enable_auto_white_balance, 1)
            
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
    
    def display_depth_stream(self, use_clahe=True):
        """
        Display depth with histogram equalization in the [dmin, dmax] meter range.

        If use_clahe=True, uses adaptive histogram equalization (CLAHE).
        """
        while True:
            latest = self.get_latest()
            if latest is None:
                continue

            depth_m = latest['depth']  
            if depth_m is None:
                continue

            dmin, dmax = self.dmin, self.dmax  

            # 1) Clip to [dmin, dmax] and ignore invalid (<=0)
            depth = depth_m.copy()
            invalid_mask = (depth <= 0) | ~np.isfinite(depth)
            depth[invalid_mask] = dmin  

            depth = np.clip(depth, dmin, dmax)

            # 2) Normalize to [0, 255] uint8
            depth_norm = (depth - dmin) / (dmax - dmin)
            depth_norm = np.clip(depth_norm, 0.0, 1.0)
            depth_u8 = (depth_norm * 255).astype(np.uint8)

            # 3) Histogram equalization
            if use_clahe:
                # Adaptive equalization (better if illumination / depth distribution is uneven)
                clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
                depth_eq = clahe.apply(depth_u8)
            else:
                # Global histogram equalization
                depth_eq = cv2.equalizeHist(depth_u8)

            depth_eq[invalid_mask] = 0

            # 4) Apply colormap
            depth_color = cv2.applyColorMap(depth_eq, cv2.COLORMAP_JET)

            cv2.imshow("Depth (hist equalized)", depth_color)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
    
    
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
    
    def plot_depth_point_cloud(self, depth_image, z_range = [0.07, 0.9], stride=8, title="Depth Point Cloud"):
        """
        Plot a 3D point cloud from a depth image.
        
        Parameters
        ----------
        depth_image : np.ndarray, shape (H, W)
            Depth image (e.g., uint16 or float32).
        stride : int
            Subsampling step to avoid plotting every pixel (for speed).
            stride=1 plots all pixels, >1 plots every Nth pixel.
        title : str
            Title for the plot.
        """
        depth = depth_image
        if depth.ndim != 2:
            raise ValueError("depth_image must be 2D (H, W)")

        H, W = depth.shape

        # Generate row/col index grids
        rows, cols = np.indices((H, W))  # rows = y, cols = x

        # Optionally downsample for speed / clarity
        rows = rows[::stride, ::stride]
        cols = cols[::stride, ::stride]
        depth = depth[::stride, ::stride]

        # Mask out invalid depths (0 or NaN)
        mask = (depth > 0) & np.isfinite(depth)

        # Depth range mask
        if z_range is not None:
            z_min, z_max = z_range
            mask &= (depth >= z_min) & (depth <= z_max)
    
        if not np.any(mask):
            print("No valid depth pixels to plot.")
            return

        x = cols[mask].ravel()
        y = rows[mask].ravel()
        z = depth[mask].astype(np.float32).ravel()

        fig = plt.figure(figsize=(7, 6))
        ax = fig.add_subplot(111, projection="3d")

        # Scatter plot
        ax.scatter(x, y, z, s=1)  # s = marker size

        ax.set_xlabel("Column (x)")
        ax.set_ylabel("Row (y)")
        ax.set_zlabel("Depth (z)")
        ax.set_title(title)

        # Optional: flip Y to match image coordinates visually
        ax.invert_yaxis()

        plt.tight_layout()
        plt.show()

        return ax
    
        

def main():
    rgbd_cam = RGBD_Cam()
    rgbd_cam.start_stream()
    
    while (not rgbd_cam.is_ready()):
        time.sleep(0.5)
    
     
    # rgbd_cam.display_depth_stream()
    while(True):
        # rgbd_cam.display_all_streams()
    #         image = rgbd_cam.get_latest()
    #         # rgbd_cam.plot_depth_point_cloud(image['depth'])
            img = rgbd_cam.get_latest()
            if img is None:
                continue 
            img = img['color']
            pixel = rgbd_cam.get_beam_pixel(img)
            print(pixel)
    #         # print(pixel)
    #         # rounded_pixel = np.rint(pixel).astype(int)  
    #         # cv2.circle(img, rounded_pixel, radius=5, color=(0, 255, 0), thickness=-1)
    #         # cv2.imshow("pixel marker", img)
        # key = cv2.waitKey(1) & 0xFF
        # if key == ord('q'):
        #     break
    

if __name__=='__main__':
    main()
    