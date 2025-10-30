import pyrealsense2 as rs
import numpy as np
import cv2

class RGBD_Cam():
    def __init__(self, width = 640, height = 480, frame_rate = 30):
        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        # Get device product line for setting a supporting resolution
        self.pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        self.pipeline_profile = self.config.resolve(self.pipeline_wrapper)
        self.device = self.pipeline_profile.get_device()
        self.device_name = str(self.device.get_info(rs.camera_info.product_line))

        self.config.enable_stream(rs.stream.depth, width, height, rs.format.z16, frame_rate)
        self.config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, frame_rate)

        self.pipeline.start(self.config)
        
        self.color_frame = np.array()
        self.depth_frame = np.array()
    
    
    def get_camera_stream(self):
        while True:
            # Wait for a coherent pair of frames: depth and color
            frames = self.pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            # Convert images to numpy arrays
            self.depth_frame = np.asanyarray(depth_frame.get_data())
            self.color_frame = np.asanyarray(color_frame.get_data())
            self.depth_shape = self.depth_frame
            self.color_shape = self.color_frame
    
    def display_color_stream(self):
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(self.depth_image, alpha=0.03), cv2.COLORMAP_JET)
        # Show images
        cv2.namedWindow('Color Stream', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('Depth Stream', depth_colormap)
        cv2.waitKey(1)
        
    def display_depth_stream(self):
        cv2.namedWindow('Color Stream', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('Depth Stream', self.color_frame)
        cv2.waitKey(1)
        
    def display_all_streams(self):
        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(self.depth_frame, alpha=0.03), cv2.COLORMAP_JET)
        # If depth and color resolutions are different, resize color image to match depth image for display
        if self.depth_shape != self.color_shape:
            resized_color_image = cv2.resize(self.color_frame, dsize=(self.depth_shape[1], self.depth_shape[0]), interpolation=cv2.INTER_AREA)
            images = np.hstack((resized_color_image, depth_colormap))
        else:
            images = np.hstack((self.color_frame, depth_colormap))

        # Show images
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', images)
        cv2.waitKey(1)

    def deinitialize_cam(self):
        self.pipeline.stop()

    