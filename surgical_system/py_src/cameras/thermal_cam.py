import time, threading, warnings, math
import PySpin
import matplotlib.pyplot as plt
import numpy as np

if __name__=='__main__':
    from camera import Camera
else:
    from .camera import Camera

class ThermalCam(Camera):    
    def __init__(self, IRFormat="TemperatureLinear10mK", height=480, frame_rate="Rate50Hz",focal_distance=0.2, pix_Per_M = 7000):
        super().__init__("Thermal Camera", 640, height, pix_Per_M)
        self.focal_distance = focal_distance
        self.IRFormat = IRFormat
        self.frame_rate = frame_rate
        self.temp_scale = 100.0      
        self.acquiring_flag = False

        try:
            ### THIS CANNOT BE INIT IN THE THREAD, LEAVE THIS HERE
            self.initialize_camera()
            self.change_IRWindowing(self.height)
            self.set_focal_distance(self.focal_distance)
            self.change_IRFormat(self.IRFormat)
            self.change_IRFrameRate(self.frame_rate)
            self.execute_nuc()
            time.sleep(2)
            self.set_acquisition_mode()
            time.sleep(1)
            self.set_acquisition_mode()
            time.sleep(0.2)
            self.set_acquisition_mode()
            
            self.thread_ready.clear()
            self.thread = threading.Thread(target=self._run, daemon=True)
            self.thread.start() 
            self._ready = True
        except:
            print("Thermal camera failed to init")
        # Just to make sure the settings are defined before receiving images
        
    def _run(self):
        t = time.time()
        print("Starting Thermal Camera Thread...")
        try:
            while(not self._ready):     
                print("Thermal cam not ready. Waiting...")
                time.sleep(1)
            while(True):
                self.thread_ready.wait()
                images = self.start_recording(1)  # returns a list of images
                # Take that image and extract temperature info
                # if len(images) == 0:
                #     continue
                if not images:          # covers [] and None
                    continue

                img0 = images[0]
                if img0 is None:
                    continue
            
                tempInfo = np.subtract(images[0]/self.temp_scale, 273.15) 
                ts = time.time() - t
                item = {"thermal": tempInfo, "ts": ts}
                self._lock.acquire()
                self._latest = item
                self._lock.release()
        finally:
            pass
    
    # def get_latest(self, timeout=None):
    #     """Return the most recent {'image','ts'} or None if nothing yet."""
    #     try:
    #         return self._q.get(timeout=timeout)
    #     except queue.Empty:
    #         return None
        
    def display(self):
        if(self.get_latest() == None):
            return
        plt.imshow(self.get_latest()['thermal'], cmap='gray')
        plt.pause(0.001)
        # Clear current reference of a figure. This will improve display speed significantly
        plt.clf()
        
    # def is_ready(self):
    #     return self._ready
    
    # def stop_stream(self):
    #     self.thread_ready.clear()
    
    # def start_stream(self):
    #     self.thread_ready.set()
        
    def initialize_camera(self):
        # Retrieve singleton reference to system object
        self.system = PySpin.System.GetInstance()

        # Get current library version
        version = self.system.GetLibraryVersion()
        print('FLIR: Library version: %d.%d.%d.%d' % (version.major, version.minor, version.type, version.build))

        # Retrieve list of cameras from the system
        self.cam_list = self.system.GetCameras()
        num_cameras = self.cam_list.GetSize()

        # Finish if there are no cameras
        if num_cameras == 0:
            # Clear camera list before releasing system
            self.cam_list.Clear()
            # Release system instance
            self.system.ReleaseInstance()
            print('FLIR: No camera!')
        else:
            # Camera to acquire images from.
            self.cam = self.cam_list[0]

            # Initialize camera
            self.cam.Init()

            # Retrieve GenICam nodemap
            self.nodemap = self.cam.GetNodeMap()
            self.nodemap_tldevice = self.cam.GetTLDeviceNodeMap()
            self.nodemap_s = self.cam.GetTLStreamNodeMap()

    def acquire_and_display_images(self, num_frames, display=False, debug=False):
        """
        This function continuously acquires images from a device and display them in a GUI.
        Modified from the PySpin Example Acquisition.py and AcquireAndDisplay.py
        set_acquisition_mode() must be run before executing this function.
        :param debug: Adds print statements and timing information for debugging
        :param num_frames: The number of frames to capture
        :param display: Whether you want the images displayed as they are captured
        :return: Returns a list of each image captured
        """
            # print('Acquiring images...')

            #  Retrieve device serial number for filename
            #
            #  *** NOTES ***
            #  The device serial number is retrieved in order to keep cameras from
            #  overwriting one another. Grabbing image IDs could also accomplish
            #  this.
            # device_serial_number = ''
            # node_device_serial_number = PySpin.CStringPtr(nodemap_tldevice.GetNode('DeviceSerialNumber'))
            # if PySpin.IsAvailable(node_device_serial_number) and PySpin.IsReadable(node_device_serial_number):
            #     device_serial_number = node_device_serial_number.GetValue()
            #     print('Device serial number retrieved as %s...' % device_serial_number)

            # # Figure(1) is default so you can omit this line. Figure(0) will create a new window every time program
            # hits this line fig = plt.figure(1)
        images = []
        if not self.acquiring_flag:
            self.set_acquisition_mode()
        if debug:
            print('FLIR: Acquisition mode set to continuous...')
            t = time.time()
        # Retrieve and display images
        for i in range(num_frames):
            try:
                #  Retrieve next received image
                #  *** NOTES ***
                #  Capturing an image houses images on the camera buffer. Trying
                #  to capture an image that does not exist will hang the camera.
                #  *** LATER ***
                #  Once an image from the buffer is saved and/or no longer
                #  needed, the image must be released in order to keep the
                #  buffer from filling up.

                image_result = self.cam.GetNextImage(1000)

                #  Ensure image completion
                if image_result.IsIncomplete():
                    print('FLIR: Image incomplete with image status %d ...' % image_result.GetImageStatus())
                else:
                    # Getting the image data as a numpy array
                    image_data = image_result.GetNDArray()
                    if display:
                        # Draws an image on the current figure
                        plt.imshow(image_data, cmap='gray')
                        plt.pause(0.001)
                        # Clear current reference of a figure. This will improve display speed significantly
                        plt.clf()


                    #  Convert image to mono 8 and append to list
                    # images.append(image_result.Convert(PySpin.PixelFormat_Mono8, PySpin.HQ_LINEAR))
                    images.append(image_data)

                #  Release image
                #  *** NOTES ***
                #  Images retrieved directly from the camera (i.e. non-converted
                #  images) need to be released in order to keep from filling the
                #  buffer.
                image_result.Release()

            except PySpin.SpinnakerException as ex:
                print('FLIR: Error: %s' % ex)
                return False
        if debug:
            elapsed = time.time() - t
            print(f"FLIR: Acquired {num_frames} frames in {elapsed} seconds.")
        #  End acquisition
        #
        #  *** NOTES ***
        #  Ending acquisition appropriately helps ensure that devices clean up
        #  properly and do not need to be power-cycled to maintain integrity.
        # self.cam.EndAcquisition()

        return images

    def set_acquisition_mode(self):
        """
        Sets the buffer handling and acquisition modes, then begins acquisition
        This needs to be called before calling acquire_and_display_images()
        """
        if not self.acquiring_flag:
            # Change bufferhandling mode to NewestOnly
            node_bufferhandling_mode = PySpin.CEnumerationPtr(self.nodemap_s.GetNode('StreamBufferHandlingMode'))
            if not PySpin.IsAvailable(node_bufferhandling_mode) or not PySpin.IsWritable(node_bufferhandling_mode):
                print('FLIR: Unable to set stream buffer handling mode.. Aborting...')
                return False

            # Retrieve entry node from enumeration node
            node_newestonly = node_bufferhandling_mode.GetEntryByName('NewestOnly')
            if not PySpin.IsAvailable(node_newestonly) or not PySpin.IsReadable(node_newestonly):
                print('FLIR: Unable to set stream buffer handling mode.. Aborting...')
                return False

            # Retrieve integer value from entry node
            node_newestonly_mode = node_newestonly.GetValue()
            # Set integer value from entry node as new value of enumeration node
            node_bufferhandling_mode.SetIntValue(node_newestonly_mode)

            node_acquisition_mode = PySpin.CEnumerationPtr(self.nodemap.GetNode('AcquisitionMode'))

            if not PySpin.IsAvailable(node_acquisition_mode) or not PySpin.IsWritable(node_acquisition_mode):
                print('FLIR: Unable to set acquisition mode to continuous (enum retrieval). Aborting...')
                return False

            # Retrieve entry node from enumeration node
            node_acquisition_mode_continuous = node_acquisition_mode.GetEntryByName('Continuous')
            if not PySpin.IsAvailable(node_acquisition_mode_continuous) or not PySpin.IsReadable(
                    node_acquisition_mode_continuous):
                print('FLIR: Unable to set acquisition mode to continuous (entry retrieval). Aborting...')
                return False

            # Retrieve integer value from entry node
            acquisition_mode_continuous = node_acquisition_mode_continuous.GetValue()

            # Set integer value from entry node as new value of enumeration node
            node_acquisition_mode.SetIntValue(acquisition_mode_continuous)

            # Disable non-uniform image correction during acquisition.
            self.disable_autoNUC()

            self.cam.BeginAcquisition()
            self.acquiring_flag = True

    def change_IRFormat(self, IRFormat):
        """
        Changes the image data type of the IR camera
        :param IRFormat: "Radiometric","TemperatureLinear100mK","TemperatureLinear10mK"
        :type IRFormat: str
        :return:
        """
        if not self.acquiring_flag:
            node_IRFormat = PySpin.CEnumerationPtr(self.nodemap.GetNode("IRFormat"))
            # print(node_IRFormat.GetIntValue())
            IRFormat_mode_value = node_IRFormat.GetEntryByName(IRFormat)
            # print(IRFormat_mode_value.GetValue())
            node_IRFormat.SetIntValue(IRFormat_mode_value.GetValue())

            print(f"FLIR: Set IRFormat to {IRFormat}.")
            time.sleep(1)
        else:
            warnings.warn("Can't change camera params while acquiring images. End acquisition first")

    def change_IRWindowing(self, height):
        """
        Changes the window size of the output image and the height. 
        If the height is bigger than window size there's an error
        :param height: 120/240/480
        :type height: int
        :return:
        """
        if not self.acquiring_flag:
            ## CHANGING IRWINDOWING MODE CAUSES US TO LOSE CONNECTION TO THE CAMERA
            node_Height = PySpin.CIntegerPtr(self.nodemap.GetNode("Height"))
            # print(node_Height.GetValue())
            node_Height.SetValue(height)
            print(f"FLIR: Set Height to {height}.")
            time.sleep(1)

            node_WindowFormat = PySpin.CEnumerationPtr(self.nodemap.GetNode("IRWindowing"))
            windowFormat_mode_value = int(math.log(480/height,2))  # 0: full window, 1: half window, 2: quarter window
            node_WindowFormat.SetIntValue(windowFormat_mode_value)
            print(f"FLIR: Set window mode to {windowFormat_mode_value}")
            time.sleep(5)

            self.execute_nuc()
        else: 
            warnings.warn("Can't change camera params while acquiring images. End acquisition first")      

    def change_IRFrameRate(self, rate):
        """
        Changes the framerate of the FLIR camera
        IR frame rate gets increased if window size changes
        :param rate: "Rate##Hz" 50,25,12,6,3 Hz
        :type rate: str
        :return:
        """
        if not self.acquiring_flag:
            node_IRFrameRate = PySpin.CEnumerationPtr(self.nodemap.GetNode("IRFrameRate"))
            # print(node_IRFrameRate.GetIntValue())

            IRFrameRate_value = node_IRFrameRate.GetEntryByName(rate)
            # print(IRFrameRate_value.GetValue())

            node_IRFrameRate.SetIntValue(IRFrameRate_value.GetValue())
            print(f"FLIR: Set IRFrameRate to {rate}.")
            time.sleep(1)
        else:
            warnings.warn("Can't change camera params while acquiring images. End acquisition first")

    def start_recording(self, num_frames):
        """
        Wrapper function for recording images
        """
        try:
            images = self.acquire_and_display_images(num_frames, display=False, debug=False)
        except PySpin.SpinnakerException as ex:
            print('Error: %s' % ex)
        return images
    
    def disable_autoNUC(self):
        if not self.acquiring_flag:
            node_NUCMode = PySpin.CEnumerationPtr(self.nodemap.GetNode("NUCMode"))
            node_NUCMode_off = node_NUCMode.GetEntryByName('Off')
            NUCMode_off = node_NUCMode_off.GetValue()
            node_NUCMode.SetIntValue(NUCMode_off)
            print("FLIR: Automatic non-uniform image correction disabled")
        else:
            warnings.warn("Can't change camera params while acquiring images. End acquisition first")

    def execute_nuc(self):
        time.sleep(1)
        node_NUCAction = PySpin.CCommandPtr(self.nodemap.GetNode("NUCAction"))
        node_NUCAction.Execute()
        time.sleep(1)

    def execute_autofocus(self,x1=260,x2=380,y1=200,y2=280):
        """
        Autofocus the Camera
        """
        # Set Area to Focus on
        node_FocusAreaX1 = PySpin.CIntegerPtr(self.nodemap.GetNode("FocusAreaX1"))
        node_FocusAreaX1.SetValue(x1)
        node_FocusAreaX2 = PySpin.CIntegerPtr(self.nodemap.GetNode("FocusAreaX2"))
        node_FocusAreaX2.SetValue(x2)
        node_FocusAreaY1 = PySpin.CIntegerPtr(self.nodemap.GetNode("FocusAreaY1"))
        node_FocusAreaY1.SetValue(y1)
        node_FocusAreaY2 = PySpin.CIntegerPtr(self.nodemap.GetNode("FocusAreaY2"))
        node_FocusAreaY2.SetValue(y2)

        # Make sure we have a fine focusing mode
        node_AutoFocusMode = PySpin.CEnumerationPtr(self.nodemap.GetNode("AutoFocusMode"))
        node_AutoFocusMode_fine = node_AutoFocusMode.GetEntryByName("Fine")
        node_AutoFocusMode.SetIntValue(node_AutoFocusMode_fine.GetValue())

        # Execute Auto Focus
        node_AutoFocus = PySpin.CCommandPtr(self.nodemap.GetNode("AutoFocus"))
        node_AutoFocus.Execute()
        time.sleep(1)

    def set_focal_distance(self, focalDistance=0.2):
        if (focalDistance > 1.0) or (focalDistance < 0.2):
            focalDistance = max(min(focalDistance,1.0),0.2)
            warnings.warn("Focal Distance outside of range [0.2,1] meters. Set to closest value")

        node_FocusDistance = PySpin.CFloatPtr(self.nodemap.GetNode("FocusDistance"))
        node_FocusDistance.SetValue(focalDistance)
        print("FLIR: Focal Distance Set to %0.2f" % focalDistance)


    def set_filter(self, filter_on=1):
        node_filter = PySpin.CEnumerationPtr(self.nodemap.GetNode("NoiseReduction"))
        node_filter.SetIntValue(filter_on)
        print("FLIR: Filter set to %d"  % filter_on)

    
    def end_acquisition(self):
        if self.acquiring_flag:
            self.cam.EndAcquisition()
            self.acquiring_flag = False

            # Enable automatic non-uniform image correction
            node_NUCMode = PySpin.CEnumerationPtr(self.nodemap.GetNode("NUCMode"))
            node_NUCMode_auto = node_NUCMode.GetEntryByName('Automatic')
            NUCMode_auto = node_NUCMode_auto.GetValue()
            node_NUCMode.SetIntValue(NUCMode_auto)

    def deinitialize_cam(self):
        # End acquisitions
        self.end_acquisition()

        # Reset IRFormat for use in FLIR software
        self.change_IRFormat('Radiometric') # This is so the camera words with ResearchIR

        # Deinitialize camera
        self.cam.DeInit()

        # Release reference to camera
        # NOTE: Unlike the C++ examples, we cannot rely on pointer objects being automatically
        # cleaned up when going out of scope.
        # The usage of del is preferred to assigning the variable to None.
        del self.cam

        # Clear camera list before releasing system
        self.cam_list.Clear()

        # Release system instance
        self.system.ReleaseInstance()
        print("FLIR: Camera Closed")

    def spam(self):
        """ Testing Method for the singleton"""
        return id(self)

    # def __init__(self, IRFormat="TemperatureLinear10mK", height=480, frameRate="Rate50Hz",focalDistance=0.2):
    #     """ Create singleton instance """
    #     # Check whether we already have an instance
    #     if ThermalCam.__instance is None:
    #         # Create and remember instance
    #         ThermalCam.__instance = ThermalCam.__impl(IRFormat=IRFormat, height=height, frame_rate=frameRate,focal_distance=focalDistance)
    #     else:
    #         print("Camera Object Already Exists.\nReturning existing object")
    #     # Store instance reference as the only member in the handle
    #     self.__dict__['_Singleton__instance'] = ThermalCam.__instance

    # def __getattr__(self, attr):
    #     """ Delegate access to implementation """
    #     return getattr(self.__instance, attr)

    # def __setattr__(self, attr, value):
    #     """ Delegate access to implementation """
    #     return setattr(self.__instance, attr, value)
    
    def __del__(self):
        self.deinitialize_cam()
        


def record_temp_over_area():
    cam_obj = ThermalCam(IRFormat="TemperatureLinear100mK", height=.4, frameRate="Rate50Hz")
    import numpy as np
    import cv2 as cv
    time.sleep(2)
    try:
        image_list = cam_obj.acquire_and_display_images(5, display=False, debug=False)
        im = (image_list[-1] - image_list[-1].min())
        im = np.array(im*255.0/im.max(),dtype=np.uint8)
        bbox = cv.selectROI('select',im)
        row_roi = [bbox[1], bbox[1]+bbox[3]]
        col_roi = [bbox[0], bbox[0]+bbox[2]]
        cv.destroyWindow('select')

        num_frames = 1
        image_list = cam_obj.acquire_and_display_images(num_frames, display=True, debug=True)

        temp_max_vec = np.zeros([num_frames,1])
        temp_min_vec = np.zeros([num_frames,1])
        temp_avg_vec = np.zeros([num_frames,1])
        i = 0
        for image in image_list:
            temp_max_vec[i] = np.max(image[row_roi[0]:row_roi[1], col_roi[0]:col_roi[1]])
            temp_min_vec[i] = np.min(image[row_roi[0]:row_roi[1], col_roi[0]:col_roi[1]])
            temp_avg_vec[i] = np.mean(image[row_roi[0]:row_roi[1], col_roi[0]:col_roi[1]])
            i = i + 1

        print("\nAverage Maximum Temperature Per Frame: ", (np.mean(temp_max_vec)/10 - 273.15))
        print("Average Minimum Temperature Per Frame: ", (np.mean(temp_min_vec)/10 - 273.15))
        print("Average Average Temperature Per Frame: ", (np.mean(temp_avg_vec)/10 - 273.15))
        print("Maximum Temperature across all frames: ", (np.max(temp_max_vec)/10 - 273.15))
        print("Minimum Temperature across all frames: \n", (np.min(temp_min_vec)/10 - 273.15))

        image = image_list[-1]
        im = (image - image.min())
        im = np.array(im*255.0/im.max(),dtype=np.uint8)
        cv.imshow('Image',im)
        cv.waitKey(0)
    finally:
        cam_obj.deinitialize_cam()


def save_image(cam_obj, name=None):
    import numpy as np
    import cv2 as cv
    import pandas as pd 

    # cam_obj.execute_autofocus()
    num_frames = 1
    # cam_obj.set_filter(1)
    image_list = cam_obj.acquire_and_display_images(num_frames, display=True, debug=True)
    cam_obj.endAcquisition
    timestr = time.strftime("%H%M%S")
    filename = "python_src/thermal_cam_control/img_processing/laserSpotFinder/" + name + timestr
    np.save(filename,image_list[-1])
    max_temp = np.max(image_list[-1])/100 - 273.15
    min_temp = np.min(image_list[-1])/100 - 273.15
    print("Max Temp %0.2f degrees Celsius" % max_temp)
    print("Min Temp %0.2f degrees Celsius" % min_temp)   
    

def npy_to_mat():
    import glob
    import os
    from scipy.io import savemat
    import numpy as np
    npy_files = glob.glob("/home/nepacheco/Repositories/Laser_control/python_src/thermal_cam_control/CoolingAgarImages/Test1/*.npy")
    npy_files = sorted(npy_files)

    time_vector = []
    surface_images = np.empty((480,640,0))
    
    for f in npy_files:
        fm = os.path.splitext(f)[0]+'.mat'
        meas_time = int(fm.split('_')[-1][0:-4])
        time_vector.append(meas_time)
        d = np.load(f)
        surface_images = np.dstack((surface_images,d))
        temp_dict = {'surfaceImage':d}
        savemat(fm,temp_dict)

    time_vector = np.array(time_vector)
    savemat("/home/nepacheco/Repositories/Laser_control/python_src/thermal_cam_control/CoolingAgarImages/Test1/AllData.mat",{'surfaceImages':surface_images,'timeVector':time_vector})



if __name__ == '__main__':
    # npyToMat()

    cam_obj = ThermalCam(IRFormat="TemperatureLinear10mK", height=480, frame_rate="Rate50Hz",focal_distance=0.20)
    # cam_obj.acquire_and_display_images(500,True)
    cam_obj.start_stream()
    while True:
        cam_obj.display()
    
    cam_obj.deinitialize_cam()      
    
    pass
