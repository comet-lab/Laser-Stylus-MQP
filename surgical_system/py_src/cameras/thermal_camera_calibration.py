import os, sys, glob
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
import matplotlib.pyplot as plt
import numpy as np 
import cv2
import yaml, csv
from thermal_cam import ThermalCam

class CameraCalibration():
    def __init__(self,calibration_folder='img_processing/CalibrationImages_0.2FD/',mode='2D',\
                 checkerboard_size=[5,5],\
                 filepath ="/home/nepacheco/Repositories/Laser_control/python_src/thermal_cam_control/", debug=False):
        self.filepath = filepath
        self.directory = "python_src/thermal_cam_control/"
        if mode not in ('2D', '3D'):
            raise Exception("Mode needs to either 2D or 3D")
        self.debug = debug
        self.mode = mode

        camera_folder = 'python_src/thermal_cam_control/' 
        filename = 'Camera_Calibration_Settings.csv'
        file_address = camera_folder + calibration_folder + filename
    

        self.checkboardSize = checkerboard_size
        if not (calibration_folder == ''):
            if os.path.exists(file_address):
                _, self.camera_matrix, self.dist_coeffs = self.read_calibration_settings(file_address)
            else:
                self.camera_matrix, self.dist_coeffs = self.calibrate(calibration_folder,checkerboard_size) 
        
    def calibrate(self,foldername,checkerboard_size):
        """
        This function will calibrate the camera using existing images in the folder to calculate the
        camera matrix and distortion coefficients.
        
        @param folderName: folder name and path where calibration images are stored 
        @param checkerBoardSize: (n,m) tuple with number of rows (n) and columns (m) in checkerboard
        
        @return cameraMatrix: camera matrix, distCoeffs: distortion coefficients
        """

        internal_size = (checkerboard_size[0]-1, checkerboard_size[1]-1)
        if foldername == '':
            # if folder is empty, we will take new calibration images
            self.save_calibration_pictures()
            foldername = 'img_processing/calibration_imgs/'
        
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 0.001)


        nrows = internal_size[0]
        ncols = internal_size[1]
        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        obj_point = np.zeros((nrows*ncols, 3), np.float32)
        obj_point[:, :2] = np.mgrid[0:nrows, 0:ncols].T.reshape(-1, 2)  #each object point corresponds to a corner in the grid
        obj_point = obj_point[:,[1,0,2]]
        # Arrays to store object points and image points from all the images.
        all_obj_points = [] # 3d point in real world space
        all_img_points = [] # 2d points in image plane.

        # load png images
        images_folder = glob.glob(self.filepath + foldername + '*.png') 
        if images_folder == []:
            # if its empty try jpeg
            images_folder = glob.glob(self.filepath + foldername + '*.jpeg')
        invalidImagesCounter = 0 # count number of images that weren't good enough for calibration
        for fname in images_folder:
            img = cv2.imread(fname)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            # Find the chess board corners
            # ret, thresh1 = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY_INV)  # Can't find corners in the thresholded images
            ret, corners = cv2.findChessboardCorners(gray, internal_size, None) 
            # If found, add object points, image points (after refining them)
            if ret == True:
                all_obj_points.append(obj_point)
                corners2 = cv2.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
                all_img_points.append(corners2)
                # Draw and display the corners
                if self.debug:
                    cv2.drawChessboardCorners(img, internal_size, corners2, ret)
                    cv2.imshow(fname, img)
                    cv2.waitKey(1000)
                    cv2.destroyAllWindows()
            else:
                invalidImagesCounter = invalidImagesCounter + 1
        
        # With all the object points and image points, run calibration
        retval, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(all_obj_points, all_img_points, gray.shape[::-1], None, None)
        # save calibration for quicker reload
        self.save_calibration_settings(retval, camera_matrix, dist_coeffs,foldername, 'Camera_Calibration_Settings.csv')
        
        if retval > 0:
            print("Succesful Calibration")
        else:
            print("Calibration was unsuccesful.")
        if self.debug:
            # Perform a distortion test on one of the calibration images
            print("%d Invalid Images" % invalidImagesCounter)
            print("Reprojection Pixel RMSE: %0.3f" % retval)
            print("Camera Matrix:\n", camera_matrix)
            print("Distortion Coefficients:\n",dist_coeffs)
            img = cv2.imread(images_folder[0])
            h, w = img.shape[:2]
            newcameramtx, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (w,h), 1, (w,h))
            dst = cv2.undistort(img, camera_matrix, dist_coeffs, None, newcameramtx)
            x, y, w, h = roi
            dst = dst[y:y+h, x:x+w]
            cv2.imshow('UndistortedImage', dst)
            cv2.waitKey(10000)
            cv2.destroyAllWindows()
        
        return camera_matrix, dist_coeffs

    def save_calibration_settings(self, retval, camera_matrix, dist_coeffs, file_location, filename):
        ''' 
        Saves camera calibration settings to a csv file.  
        
        @param retval: reprojection error
        @param cameraMatrix: camera matrix
        @param distCoeffs: distortion coefficients
        @param fileLocation: folder location to save the file
        @param fileName: name of the file
        
        @return None
        '''
        ret = [[retval]]
        folder_name='python_src/thermal_cam_control/'
        try:
            file_location = os.path.join(folder_name + file_location, filename)
            with open(file_location, 'w', newline='') as file:
                writer = csv.writer(file)
                writer.writerows(camera_matrix)
                writer.writerows(dist_coeffs)
                writer.writerows(ret)
        except:
            print("Directory was not found")
            return 0
        print("Saving Camera Calibration Settings as: ",folder_name + file_location, filename)

    def read_calibration_settings(self, file_location):
        '''
        Reads camera calibration settings from a csv file.
        
        @param fileLocation: folder location to read the file
        
        @return retval: reprojection error, cameraMatrix: camera matrix, distCoeffs: distortion coefficients
        '''
        settings = []
        camera_matrix = []
        dist_coeffs = []
        print("Reading Exisiting Camera Calibration Settings...")

        try:
            with open(file_location, 'r', newline='') as file:
                reader = csv.reader(file)
                for row in reader:
                    if row:
                        settings.append([float(i) for i in row])
            for row in range(0,3):
                camera_matrix.append(settings[row])
                
            camera_matrix = np.array(camera_matrix, dtype=np.float64)
            dist_coeffs.append(settings[3])
            dist_coeffs = np.array(dist_coeffs, dtype=np.float64)
            retval = settings[4][0]
        except:
            print("File was not found in the directory or CSV formatting is incorrect.")
            return 0,0,0 #Set readings as 0 
        
        if self.debug:
            print('Camera Matrix:')
            print(camera_matrix)
            print("disCoeffs: ", dist_coeffs )
            print("retval: ", retval)
            

        return retval, camera_matrix, dist_coeffs

    def generate_transform(self, imgpts, objpts):
        '''
        This function will generate the 3D transform matrix and homography matrix for image warping.
        
        @param imgPts: numpy array [2 x n] of image points (points in source image)
        @param objPts: numpy array [3 x n] of object points (sometimes world coordinates)
        @param M_pix_per_m: scale factor with which to scale object points for 2D homography
        
        @return None
        '''
        self.imgpts = imgpts.copy()
        self.objpts = objpts.copy()

        # Generate the 3D transform.
        retval, self.rvec, self.tvec = cv2.solvePnP(self.objpts, self.imgpts, self.camera_matrix, self.dist_coeffs)
        
        # Create the homography matrix for image warping. For visualization, scale obj_points by M_scale so that 
        # we work in two pixel coordinates instead of pixel coordinates for img_points and meters for object_points

        # first remove the lens distortion - we stopped removing lens distortion because it makes it impossible to use M in MATLAB
        # _imgPts = cv2.undistortPoints(_imgPts,self.cameraMatrix,self.distCoeffs)
        # _imgPts = _imgPts[:,0,:] # undistortPoints returns each [u,v] point as [[u,v]]. So we need to shrink back to 2D array
        # # 1: transpose image points. 2: append 1 to last row. 3: multiply by camera matrix to get back to pixesl
        # # 4: tranpose again. 
        # _imgPts = np.transpose(np.matmul(self.cameraMatrix,np.concatenate((_imgPts.T,np.array([[1,1,1,1,1]])))))
        # have to make sure both are float32
        _imgpts = np.array(self.imgpts, np.float32) # already is [2xn]
        _objpts = np.array(self.objpts[:,0:2],np.float32) # need this to be [2 x n] 
        # findHomography is the same as getPerspectiveTransform but it can take more than 4 points, 
        # and they don't have to be a quadrilateral. 

        self.M,_ = cv2.findHomography(_imgpts,_objpts)
    
    def world_to_pixel(self, world_coords):
        """
        Takes a nx3 world point and converts it to a nx2 pixel coordinate
        
        @param world_coords: nx3 numpy array of world coordinates
        
        @return pixel_coords: nx2 numpy array of pixel coordinates
        """
        _world_coords = world_coords.copy() # Need to make a copy or python will edit world_coords outside of function because of pass by reference
        world_shape = _world_coords.shape
        if len(world_shape) == 1:
            _world_coords = _world_coords.reshape([1,3])
            world_shape = _world_coords.shape
        if world_shape[1] != 3 and world_shape[0] == 3:
            _world_coords = _world_coords.T

        world_shape = _world_coords.shape
        # imgpnt is the pixel coordinates [x,y] of a real world point (given in pnt)
        if self.mode == '3D':
            pixel_coords, _ = cv2.projectPoints(_world_coords, self.rvec, self.tvec, self.camera_matrix, self.dist_coeffs)
            # this function returns a matrix that is [n x 1 x 2]
            pixel_coords = pixel_coords.reshape([-1,2]) # reshape to [n x 2]
        elif self.mode == '2D':
            # We want to be working in 2D (planar) homogenous coordinates a.k.a. [x,y,1]
            _world_coords[:,2] = np.ones([world_shape[0]])  # shape is still [n x 3]
            # because this was generated with a scale we need to scale it again
            # double transpose so matrix multiplication is done with [3 x n] and then returns [nx3]
            pixel_coords = np.matmul(np.linalg.inv(self.M),_world_coords.T).T
            # this is to handle the scale factor [u',v',t] -> [u,v,1] and convert to pixel homogenous coordinates
            pixel_coords = pixel_coords/(pixel_coords[:,2].reshape([-1,1])) # element-wise division
            # Distort the pixels to match camera image 
            # pixel_coords = self.applyDistortion(pixel_coords)
            pixel_coords = pixel_coords[:,0:2]
        if self.debug:
            for i in range(pixel_coords.shape[0]):
                print("Pixel Location from world point: ",pixel_coords[i,0:2])
                
        return pixel_coords

    def pixel_to_world(self,pixel_coords,z_input=0):
        '''
        Converts pixel coordinates to world coordinates.
        
        @param pixel_coords: nx2 numpy array of pixel coordinates
        @param z_input: [nx1] array of z heights of the world coordinates
        
        @return worldPoint: nx3 numpy array of world coordinates
        '''
        _pixel_coords = pixel_coords.copy() # Python is pass by reference so we don't want to make changes to input coords.
        pixel_shape = _pixel_coords.shape
        if len(pixel_shape) == 1:
            _pixel_coords = _pixel_coords.reshape([1,2])
            pixel_shape = _pixel_coords.shape
        if pixel_shape[1] != 2 and pixel_shape[0] == 2:
            _pixel_coords = _pixel_coords.T

        pixel_shape = _pixel_coords.shape
        if np.isscalar(z_input):
            z_input = z_input * np.ones([pixel_shape[0]])

        if self.mode == '3D':
            # find rotation matrix using Rodrigues
            Rot, _ = cv2.Rodrigues(self.rvec)
            Rinv = np.linalg.inv(Rot)

            #Equation we are solving:
            # s*[u;v;1] = K*[R | T]*[x;y;z;1]
            # s * R^-1 * K^-1 *[u;v;1] = [x;y;z] + R^-1*T

            # basically 'Undistort' performs the K^-1*[u,v,1] calculation, where Kinv is the inverse of the
            # camera matrix. It also undistorts the image using the distortion coefficients so is more accurate
            # than just using np.linalg.inv(self.cameraMatrix)*pixel_coords
            Kinv_uv = cv2.undistortPoints(_pixel_coords, self.camera_matrix, self.dist_coeffs)
            # add the third element to make [u,v] to [u,v,1]
            # at this point we have changed to [3xn]
            Kinv_uv = np.vstack((Kinv_uv.T.reshape([2,-1]),np.ones([1,pixel_shape[0]]))) 
            # multiply Rinv to Kinv_uv to start to rotate the image to the world coordinates
            Rinv_Kinv_uv = np.matmul(Rinv,Kinv_uv)
            # find Rinv_T, where T is the translation between camera and world frame 
            Rinv_T = np.matmul(Rinv, self.tvec)

            # We know we are working on the z = 0 plane. This means the third element on both sides of 
            # the equation written above need to be equal, and we know the value on the right size. 
            # this allows us to solve for the scalar so we set z = 0 as a default argument.
            z = z_input
            s = (z + Rinv_T[2])/Rinv_Kinv_uv[2,:]
            if self.debug:
                for i in range(_pixel_coords.shape[0]):
                    print("s: %0.2f" % s[i])
            # Now the world coordaintes [x;y;z] are calculated by introducing s, and solving the above eqauation
            world_point = s*Rinv_Kinv_uv - Rinv_T
            world_point = world_point.T # put back into [nx3]
        elif self.mode == '2D':
           
            # undistort the pixel coordinates
            # pixel_coords = cv2.undistortPoints(pixel_coords,self.cameraMatrix,self.distCoeffs)
            # pixel_coords = pixel_coords[0,0,:] # undistortPoints returns each [u,v] point as [[u,v]]. So we need to shrink back to 2D array
            # convert pixel coordinates to homogenous coordinates
            # Size is now n x 3
            _pixel_coords = np.hstack([_pixel_coords,np.ones([pixel_shape[0],1])])
            #  multiply by camera matrix to get back to pixels from normalized coordinates
            # pixel_coords = np.matmul(self.cameraMatrix,pixel_coords)
 
            world_point = np.matmul(self.M,_pixel_coords.T).T # double transpose to do the math with [3xn] and convert back to [nx3]
            # remove scale factor in world point and make into homogenous coordinates
            world_point = world_point/(world_point[:,2].reshape([-1,1])) # element-wise division
            # convert 2D homogenous coordinates into 3D coordinates where z height is 0
            world_point[:,2] = z_input.reshape([-1]) # the z height is always assumed to be 0
        if self.debug:
            for i in range(world_point.shape[0]):
                print("World point from pixel location: ",world_point[i,:])
        return world_point

    def change_image_perspective(self,image, scale = 1, marker = None):
        '''
        This function will change the perspective of the image to a top down view. It will perform any
        necessry shifting to guarantee the full input image is displayed in the output window. Additionally, 
        a scale variable can be included which will scale the size of the output image. 

        @param image: the image to be displayed or filename for image
        @param scale: scale factor for mapped world coordinate system
        @param marker: locations to place markers
        
        @return None
        '''

        image = CameraCalibration.process_input_image(image)
        
        im_shape = image.shape
        # get corners of our image in pixel frame; convert row,col to u,v
        corners = np.array([[0,0],[0,im_shape[0]],[im_shape[1],0],[im_shape[1],im_shape[0]]])
        # get corners in our warped frame (aka world frame)
        warped_corners = self.pixel_to_world(corners,0)
        # get the most negative value in both x and y to shift M 
        x_shift = np.min(warped_corners[:,0])
        y_shift = np.min(warped_corners[:,1])
        # calculate width and height of new shifted image
        width = np.max(warped_corners[:,0] - x_shift)
        height = np.max(warped_corners[:,1] - y_shift)

        M_T = self.M.copy()
        M_T[:,2] = M_T[:,2] - np.array([x_shift,y_shift,0])
        M_T[0:2,:] = M_T[0:2,:]*scale # this will essentially act as if we scaled the object points. 

        # have to scale width and height
        dst = cv2.warpPerspective(image,M_T,(round(width*scale),round(height*scale)))
        
        if marker is not None:
            mk = np.array(marker, dtype=np.float32)
            if mk.ndim == 1:
                mk = mk[None, :]  # make it Nx2
            pts = mk.reshape(-1, 1, 2)
            warped_pts = cv2.perspectiveTransform(pts, M_T).reshape(-1, 2)

            # Plot the marker(s)
            plt.scatter(warped_pts[:, 0], warped_pts[:, 1], marker='o', s=60, facecolors='none', edgecolors='red', linewidths=2)
        
        if self.debug:
            plt.imshow(dst,cmap='gray')
            plt.gca().invert_yaxis()
            plt.xlabel("X axis")
            plt.ylabel("Y axis")
            plt.show(block=True)
            # cv2.imshow("Warped Image",dst)
            # cv2.waitKey(0)   # waits for a key press
            # cv2.destroyAllWindows()
        return dst

    def apply_distortion(self,u_pixel_coords):
        """
        Takes in a 3x1 vector of undistorted Pixel coordinates and applies distortion to them.
        
        @param uPixelCoords: 3x1 numpy array of undistorted pixel coordinates
        
        @return distPixelCoords: 3x1 numpy array of distorted pixel coordinates
        """
        norm_pixel = np.matmul(np.linalg.inv(self.camera_matrix),u_pixel_coords)
        x = norm_pixel[0]
        y = norm_pixel[1]
        r = np.sqrt(x**2 + y**2)
        k1 = self.dist_coeffs[0,0]
        k2 = self.dist_coeffs[0,1]
        p1 = self.dist_coeffs[0,2]
        p2 = self.dist_coeffs[0,3]
        k3 = self.dist_coeffs[0,4]
        x_dist = x*(1 + k1 * r**2 + k2 * r**4 + k3 * r**6) + (2 * p1 * x * y + p2 * (r**2 + 2 * x**2))
        y_dist = y*(1 + k1 * r**2 + k2 * r**4 + k3 * r**6) + (p1 * (r**2 + 2 * y**2) + 2 * p2 * x * y)
        dist_pixel_coords = np.matmul(self.camera_matrix,np.array([x_dist, y_dist,1]))
        return dist_pixel_coords

    def select_image_points(self,image,npoints):
        '''
        This function will display an image and allow the user to select points on the image.
        @param numPoints: number of points to select
        @param fname: file name of the image to display
        
        @return points: numpy array of the selected points
        '''
        # Load your image
        image = CameraCalibration.process_input_image(image)
            
        # Display the image
        plt.imshow(image,cmap='gray')

        # Select pixel points interactively
        points = plt.ginput(n=npoints, timeout=0)

        # Print the selected pixel coordinates
        if self.debug:
            for point in points:
                x, y = point
                print(f"Pixel coordinates: x={x}, y={y}")
        plt.close()
        return np.array(points)

    def load_homography(self, M_pix_per_m = 7000, \
                         fileLocation = "/python_src/thermal_cam_control/img_processing/Laser_Alignment_Test", debug = True):
    
        img_points = CameraCalibration.load_pts(fileLocation + "laser_spots.csv")
        obj_points = CameraCalibration.load_pts(fileLocation + "laser_world_points.csv")
        
        # self.generate_transform(img_points, obj_points)
        if obj_points.shape[1] == 2:
            obj_points = np.hstack((obj_points, np.zeros((obj_points.shape[0], 1))))  # Add Z coordinate as 0
        obj_points = obj_points*M_pix_per_m
        self.generate_transform(img_points,obj_points)
        
        # project all image points through M
        projected_points = self.pixel_to_world(img_points)

        err  = np.linalg.norm(projected_points - obj_points, axis=1)
        rmse  = np.sqrt((err**2).mean())   

        print(f"RMS reprojection error = {rmse:.3f} pixels")
        print(f"RMS reprojection error = {rmse/M_pix_per_m*1000:.3f} mm")
        return rmse

    def display_2d_projection(self):

        obj_points = self.objpts.copy()
        proj_points = self.pixel_to_world(self.imgpts)

        err  = np.linalg.norm(proj_points - obj_points, axis=1)
        rmse  = np.sqrt((err**2).mean())  

        plt.scatter(obj_points[:,0], obj_points[:,1], c='b', s=50, label='Object Points')
        plt.scatter(proj_points[:,0], proj_points[:,1], c='g', s=50, label='Projected Points')
        plt.legend()
        plt.title(f'Reprojection vs. ground truth  RMS={rmse:.2f} pixels')
        plt.show(block=False)
        plt.pause(1)
        plt.close()

    def update_objpts_imgpts(self, objpt_list, imgpt_list, filename):
        """
        Saves the object point list and img point list to the filename
        
        @param objPtsList: numpy array of object points
        @param imgPtsList: numpy array of image points
        @param fileName: name of the file to save the points to
        
        @return None
        """
        # Convert the arrays to python list
        obj_points = objpt_list.tolist()
        img_points = imgpt_list.tolist()
        
        with open(self.filepath + filename, 'r') as file:
            data = yaml.load(file, Loader=yaml.FullLoader)

        # update objectPoints and imagePoints
        data['objectPoints'] = obj_points
        data['imagePoints'] = img_points

        # write updated contents to YAML file
        with open(self.filepath + filename, 'w') as f:
            yaml.dump(data, f)


    def save_to_csv(self, filename, imgpts):
        '''
        Saves the image centers to a csv file.
        
        @param filename: name of the file to save the image centers
        @param imgPoints: list of image centers
        
        @return None
        '''
        with open(self.directory + filename, 'w', newline='') as csvfile:
            csvwriter = csv.writer(csvfile)
            for i in range(len(imgpts)):  
                csvwriter.writerow(list(imgpts[i]))

    def pair_objpts_imgpts(self, obj_points, img_points):
        '''
        This function pairs the object points with the image points based on the distance from the origin.
        Disclaimer: This function is not perfect and may not work for all cases.
        
        @param objPoints: numpy array of object points
        @param imagePoints: numpy array of image points
        
        @return sortedImgPoints: numpy array of sorted image points
        '''
        frame_height = 480
        new_img_frame = np.copy(img_points)
        new_img_frame[:, 1] = frame_height - new_img_frame[:, 1]
       
        obj_distances = np.linalg.norm(obj_points[:, :2], axis=1)
        img_distances = np.linalg.norm(new_img_frame[:, :2], axis=1)
       
       
        # Get sorted indices based on distances
        sorted_obj_distances = np.argsort(obj_distances)
        sorted_img_distances = np.argsort(img_distances)
       
       
        sorted_obj_points = obj_points[sorted_obj_distances]
        img_points = img_points[sorted_img_distances]
        obj_distances = obj_distances[sorted_obj_distances]
        img_distances = img_distances[sorted_img_distances]
       
        obj_pairs = {}
        for i in range(len(obj_points) - 1):
            if abs(img_distances[i] - img_distances[i+1]) < 20:
                print("Difference from origin is too close. Conducting secondary sort.")
                if sorted_obj_points[i][0] > sorted_obj_points[i+1][0] and img_points[i][0] < img_points[i+1][0]:
                    temp = sorted_obj_points[i].copy()
                    sorted_obj_points[i] = sorted_obj_points[i+1]
                    sorted_obj_points[i+1] = temp
            obj_pairs[tuple(sorted_obj_points[i])] = img_points[i]
        obj_pairs[tuple(sorted_obj_points[-1])] = img_points[-1]
       
        # for i in range(len(objPoints)):
        #     print("objPoint: ", sortedObjPoints[i], " imgPoint: ", imagePoints[i], " objDistance: ", objDistances[i], " imgDistance: ", imgDistances[i])
       
        sorted_img_points = []
        for objpt in obj_points:
            sorted_img_points.append(obj_pairs[tuple(objpt)])
            print("objPoint: ", objpt, " imgPoint: ", obj_pairs[tuple(objpt)])
       
        return np.array(sorted_img_points)
    
    def draw_centers(self, img_path, centers):
        '''
        This function will draw the centers on the image.
        
        @param imagePath: path to the image
        @param centers: list of centers
        
        @return None
        '''
        img = cv2.imread(self.directory + img_path)
        int_centers = np.array(centers, dtype=int)  
        i = 0
        for center in int_centers:
            cv2.circle(img, tuple(center), 5, (i*10, i * 5, 255 - i * 3 ), -1)
            i += 10
        cv2.imshow("centers", img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    @staticmethod
    def process_input_image(input_variable):
        '''This function takes an input and determines if its already a 2D numpy matrix
            or if its a jpeg/png filename to load for the image. 
        '''
        # if input variable is already the np.array
        if isinstance(input_variable, np.ndarray):
            image = input_variable
        elif isinstance(input_variable, str):  # file path as string
            image = np.loadtxt(input_variable)   # or np.load, np.genfromtxt, depending on your file format
        else:
            raise TypeError("Input must be a NumPy array or a filename (string).")
        # now do something with arr
        return image

    @staticmethod
    def load_pts(filepath):
        '''
        This function reads the image points from a csv file
        
        @param filePath: path to the csv file
        
        @return imgPoints: numpy array of image points
        '''
        if os.path.exists(filepath):
            with open(filepath, 'r', newline='') as file:
                reader = csv.reader(file)
                imgPoints = [row for row in reader]    
            return np.array(imgPoints, dtype=np.float32)
        else:
            print("CSV file not found: ", filepath)
            return None  

    @staticmethod
    def get_thermal_image(thermal_cam_obj):
        '''
        This function will return a single image from the thermal camera with increased contrast.
        
        @return image: numpy array of the image
        '''
        # Thermal camera data is using a unit16 (range of 0-65536). To save an as image that has good contrast, we need to 
        # scale the range of values in the image, to a range of values between 0 and 255. For any sort of calibration
        # we probably want a large contrast, but we can also force a certain bounds if we want.
        images = thermal_cam_obj.acquire_and_display_images(1,display=False)
        image = images[0]
        min_val = np.min(image)
        max_val = np.max(image)
        image = (image - min_val)/(max_val - min_val) * 255
        return np.uint8(image)
    
    @staticmethod
    def save_calibration_pictures(thermal_cam_obj,folder_name='img_processing/calibration_imgs/',\
                                 num_images=10, focal_distance=0.2, display=True, confirm_image=True):
        """
        This function will store images to be used for checkerboard calibration. The images are automaticall scaled so that 
        the coldest part of the image is 0, and the hottest part of the image is 255.
        
        @param folderName: folder name to store the images
        @param num_images: number of images to store
        @param focalDistance: focal distance of the thermal camera
        @param display: display the image
        @param confirm_image: confirm the image is good
        
        @return None
        """
        thermal_cam_obj.setFocalDistance(focal_distance)
        # create the folder to store the images if the folder does not exist
        if not os.path.exists(folder_name):
            os.makedirs(folder_name)
        stored_images = 0
        # we want to keep trying to grab images until we hit the desired number
        if confirm_image:
            print("Press Enter or Space to accept the image and Esc to retake the image")
        while stored_images < num_images:
            # grab an image from the thermal cam
            image = CameraCalibration.get_thermal_image(thermal_cam_obj)

            if display or confirm_image: # display the image in a figure
                # Draws an image on the current figure
                cv2.imshow('Figure',np.uint8(image))
            
            # if the user wants to confirm the image is good
            if confirm_image:
                pressedKey = cv2.waitKey(0)
                if pressedKey == 13 or pressedKey == 32: # Enter is 13 and Space is 32
                    cv2.imwrite(folder_name + "image_" + str(stored_images) + ".jpeg",image)
                    stored_images = stored_images + 1
            else:  
                cv2.imwrite(folder_name + "image_" + str(stored_images) + ".jpeg",image)
                stored_images = stored_images + 1
        # Stop thermal camera
        thermal_cam_obj.endAcquisition()

    @staticmethod
    def find_checkerboard_corners(img, gridsize: tuple):
        '''
        This function finds the the corners of each square in the chessboard minus the border
    
        @param img: Image with checkerboard
        @param gridSize: The size of the grid of the chessboard
    
        @return corners: The corners of each square in the chessboard minus the border
        '''
        print("Finding Chessboard Corners...")
    
        # Find the chessboard corners
        gridsize = (gridsize[0] - 1, gridsize[1] - 1)
        ret, corners = cv2.findChessboardCorners(img, gridsize)
    
        if ret:         
                  
            if np.linalg.norm(corners[0][0]) > np.linalg.norm(corners[-1][0]):
                print("Flipping")
                corners = np.flip(corners, 0)
            img = cv2.drawChessboardCorners(img, gridsize, corners, ret)
            corners = corners.reshape(-1, 2)
            cv2.imshow('Chessboard with Centers', img)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
            return corners
        
        return None
    
    @staticmethod
    def generate_checkerboard_objpts(row, col, square_size, xoffset = 0, yoffset = 0, zoffset = 0):
        '''
        This function generates the object points for the chessboard. If row or col is negative,
        the function will decrement rather than increment. Offset is the
        starting location of the chessboard
        '''
        obj_points = np.empty((0, 3))
        xinc, yinc = col > 0, row > 0 
        row, col = abs(row), abs(col)
        
        for i in range(row - 1):
            for j in range(col - 1):
                x = j*square_size + xoffset if xinc else -j*square_size + xoffset
                y = i*square_size + yoffset if yinc else -i*square_size + yoffset
                obj_points = np.vstack((obj_points, [x, y, zoffset]))
        obj_points[:, 1] = np.flip(np.array(obj_points)[:, 1], 0) # Flip the y axis so that the first point is at the top left
        return obj_points
    
    
def test_checkerboard_transforms(mode = '2D'):
    nrows = 8
    ncols = 11
    # Assuming the calibration images that Bhaavin took are good
    filepath = "/home/nepacheco/Repositories/Laser_control/python_src/thermal_cam_control/"
    cali_obj = CameraCalibration('img_processing/CalibrationImages_0.2FD/',mode,[nrows,ncols],filepath,True)
    # Redo the objectPoint to imagePoint mapping
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 0.001)

    pix_per_cm = 70
    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    obj_points = np.zeros(((nrows-1)*(ncols-1), 3), np.float32)
    obj_points[:, :2] = np.mgrid[0:(nrows-1), 0:(ncols-1)].T.reshape(-1, 2) * pix_per_cm/2  #each object point corresponds to a corner in the grid
    obj_points = obj_points[:,[1,0,2]] # Flip first two columns because we need (x,y) not (row,col)
    fname = "img_processing/testImages/Checker4-15.jpeg"
    img = cv2.imread(cali_obj.filepath + fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    img_points = CameraCalibration.find_checkerboard_corners(gray,(nrows,ncols))

    cali_obj.generate_transform(img_points,obj_points)

    # Measure Reprojection Error
    proj_img_points = cali_obj.world_to_pixel(obj_points)
    error_pix = np.linalg.norm(img_points - proj_img_points, axis=1)

    proj_world_points = cali_obj.pixel_to_world(img_points)
    error_world = np.linalg.norm(obj_points - proj_world_points, axis=1)

    # calcute rmse
    rmse_pix = np.sqrt(np.square(error_pix).mean())
    rmse_world = np.sqrt(np.square(error_world).mean())
    print("Reprojection pixel RMSE: %0.4f pixels" % rmse_pix)
    print("Reprojection world RMSE: %0.4f mm" % (rmse_world/pix_per_cm))
    cali_obj.change_image_perspective(gray,1,marker=img_points)


if __name__=="__main__":
    test_checkerboard_transforms()
