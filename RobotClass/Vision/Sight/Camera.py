#import needed libraries
import os
import json
import picamera2
import libcamera
import numpy as np
import time
import cv2
from Vision.Sight.Filters.ColorFilter import ColorFilter

class Camera():
    def __init__(self, camera_ID=None):
        #Calibrate the camera
        self.calibration_data = None
        if(camera_ID == None):
            print("Warning: Starting camera without calibration! Please give camera ID to calibrate!")
        else:
            print("\n=========================")
            print("Gathering Camera Calibration Parameters")
            print("=========================")
            print("Calibrating Camera...")
            file_directory_path = os.path.dirname(os.path.abspath(__file__))
            calibration_folder_path = os.path.join(file_directory_path, "Camera_Calibration_Sets")
            calibration_folder_path = os.path.join(calibration_folder_path, f"Cam{camera_ID}")
            self.isCalibrated = self._calibrate_camera(calibration_folder_path)
            if(self.isCalibrated):
                print("Camera Calibrated!")
            else:
                print("Camera Calibration Failed!")

        #Instantiate the pi-camera object
        print("\n=========================")
        print("Initiallizing Camera")
        print("=========================")
        self.picam2 = picamera2.Picamera2()
        #display the different raw sensing modes
        print("\n=========================")
        print("Display Camera's Different Sensing Modes")
        print("=========================")
        for mode in self.picam2.sensor_modes:
            print(f"> {mode}")
        #Now configure the camera based on sensing mode and crop
        print("\n=========================")
        print("Configuring Camera Sensing Mode")
        print("=========================")
        #Specify the full diagonal FOV ???and focal length??? of the camera
        self.FOV_diagonal_full_deg = 160
        self.focal_length_mm = -1###3.15 #???Not sure what it is
        ###############SHOULD MAYBE USE OPEN CV CAMERA CALIBRATION ON CHECKERBOARD!!!
        print(f"Camera Full Diagonal FOV: {self.FOV_diagonal_full_deg} degrees")
        print(f"Camera Focal Length: {self.focal_length_mm} mm")
        #specify the sensing mode for the image
        self.sensing_mode = 3 #0 (bin), 1 (bin), 2 (crop), 3(bin) for the fisheye lens camera
        print(f"Using sensing mode: {self.sensing_mode}")
        print(f"> {self.picam2.sensor_modes[self.sensing_mode]}")
        #save width and height of full image for camera
        full_size_sensing_mode = self.picam2.sensor_modes[self.sensing_mode]
        self.width_full = full_size_sensing_mode["size"][0] #2592 max for fisheye cam
        self.height_full = full_size_sensing_mode["size"][1] #1944 max for fisheye cam
        #set image crop
        self.width = 2592#640#320
        self.height = 1944#480#280
        print(f"Main stream image size: (W={self.width}, H={self.height})")
        #compute new FOV for diagonal, x, and y directions from new image width
        dim_full_wh = np.array([self.width_full, self.height_full])
        dim_crop_wh = np.array([self.width, self.height])
        new_FOV_dwh = self.__compute_new_FOV(full_dim_wh=dim_full_wh, crop_dim_wh=dim_crop_wh, FOV_diagonal_full_deg=self.FOV_diagonal_full_deg)
        self.FOV_diagonal_deg = new_FOV_dwh[0]
        self.FOV_width_deg = new_FOV_dwh[1]
        self.FOV_height_deg = new_FOV_dwh[2]
        print(f"New Camera FOV: (FOV_D={self.FOV_diagonal_deg:.3f} degrees, FOV_W={self.FOV_width_deg:.3f} degrees, FOV_H={self.FOV_height_deg:.3f} degrees)")
        #set configuration for output streams
        config = self.picam2.create_preview_configuration(
            #main stream
            main = {
                "size": (self.width, self.height), #width x height
                "format": "XRGB8888" #8-bit [B, G, R, 255]
            },

            #raw stream
            raw = self.picam2.sensor_modes[self.sensing_mode],
            
            #video controls
            controls = {
                "FrameDurationLimits": (0, 1000000) #microseconds boundary
            },

            #other parameters
            transform = libcamera.Transform(hflip=True, vflip=False), #params set so that the positive and negative axis are in right direction
            queue = True,
            display = None #None, "main", or "lores"
        )
        #apply the configuration to the camera object
        self.picam2.configure(config)
        #start the camera
        print("Starting up camera...")
        self.picam2.start()
        #wait a second after starting to allow camera to fully start up
        time.sleep(1)
        print("Camera startup sequence complete!")

        #Instantiate the color filter object
        self.color_filter = ColorFilter()

    #calibrate the camera by obtaining a JSON file or a list of images to create a JSON
    def _calibrate_camera(self, calibration_folder_path, checkerboard_inner_corners=(4,4)):
        #First try to find the JSON file
        try:
            #load the JSON file if found
            calibration_file_path = os.path.join(calibration_folder_path, "Camera_Parameters.JSON")
            with open(calibration_file_path, 'r') as calibration_file:
                print(f"Calibration data found in JSON file: {calibration_file_path}")
                self.calibration_data = json.load(calibration_file)
                #convert list data to numpy arrays
                for key in self.calibration_data.keys():
                    self.calibration_data[key] = np.array(self.calibration_data[key])
            #Camera successfully calibrated, so return true
            return(True)
        #If the JSON File doesn't exist, find and use png images in the calibration folder
        except:
            #referencing: https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
            print(f"No calibration JSON file found! Attempting to calibrate camera via checkerboard images...")
            # termination criteria
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            # prepare object points: (0,0,0), (1,0,0), (2,0,0) ....,(4,4,0)
            object_point = np.zeros((checkerboard_inner_corners[0]*checkerboard_inner_corners[1],3), np.float32)
            object_point[:,:2] = np.mgrid[0:checkerboard_inner_corners[0], 0:checkerboard_inner_corners[1]].T.reshape(-1, 2)
            #setup initial variables used in loop
            object_point_list = [] # 3d point in real world space
            image_point_list = [] # 2d points in image plane.
            image_index = 0
            try:
                while(1):
                    #read the image and convert it to grayscale
                    image_path = os.path.join(calibration_folder_path, f"calibration_img{image_index}.png")
                    print(f"Reading image {image_index} from: {image_path}")
                    image = cv2.imread(image_path)
                    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                    print(f"Image {image_index} found!")
                    #potential fix for detecting checkerboard recerencing: https://stackoverflow.com/questions/66225558/cv2-findchessboardcorners-fails-to-find-corners
                    #hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
                    #lower_bound = np.array([0, 0, 145])
                    #upper_bound = np.array([180, 60, 255])
                    #mask = cv2.inRange(hsv_image, lower_bound, upper_bound)
                    #find the corners of the chessboard
                    ret, corners = cv2.findChessboardCorners(gray_image, checkerboard_inner_corners, flags=None)
                    #see if the image was found
                    if(ret):
                        #If found, add object point and image point
                        object_point_list.append(object_point)
                        image_point = cv2.cornerSubPix(gray_image, corners, winSize=(11,11), zeroZone=(-1,-1), criteria=criteria)
                        image_point_list.append(image_point)
                    #if corners on calibration image were not found, then indicate to the user
                    else:
                        print(f"Calibration of image {image_index} failed to be found!")
                        raise Exception(f"Calibration error on image {image_index}")
                    #increment image index by 1
                    image_index += 1
            except Exception as e:
                #if images were successfully parsed, solve for calibration constants
                if(image_index > 0):
                    print(f"{image_index} calibration images found in folder: {calibration_folder_path}")
                    print("Computing calibration parameters...")
                    ret, camera_matrix, distortion_coefficients, rotation_vectors, translation_vectors = cv2.calibrateCamera(object_point_list, image_point_list, imageSize=gray_image.shape[::-1], cameraMatrix=None, distCoeffs=None)
                    #if calibration parameters successfully computed, save them to a JSON
                    if(ret):
                        print("Calibration parameters successfully found!")
                        #save parameters to a JSON file
                        calibration_data_JSON = {
                            "distortion_coefficients" : distortion_coefficients.tolist(),
                            "camera_matrix" : camera_matrix.tolist(),
                        }
                        with open(calibration_file_path, 'w') as calibration_file:
                            json_string = json.dump(calibration_data_JSON, calibration_file, indent=2)
                        #read and save the data from the JSON file int the camera object
                        with open(calibration_file_path, 'r') as calibration_file:
                            self.calibration_data = json.load(calibration_file)
                            #convert list data to numpy arrays
                            for key in self.calibration_data.keys():
                                self.calibration_data[key] = np.array(self.calibration_data[key])
                        #Camera successfully calibrated, so return true
                        return(True)
                    #if calibration parameters failed to be found, indicate it to the user
                    else:
                        print(f"Calibration parameters could not be found!")
                        #Camera did not successfully calibrate, so return false
                        return(False)
                        #raise Exception(f"Calibration computation error on image set!")
                #if images were not successfully parsed, indicate it to the user
                else:
                    print("Error! No calibration file or images found! Aborting calibration process!")
                    #Camera did not successfully calibrate, so return false
                    return(False)
        #catch all return statement (assume unsuccessful calibration)
        return(False)

    #compute the new FOV of the diagonal, x, and y from a crop
    def __compute_new_FOV(self, full_dim_wh, crop_dim_wh, FOV_diagonal_full_deg):
        #dwh = diagonal, width, height
        len_full_dwh = np.insert(full_dim_wh, 0, np.linalg.norm(full_dim_wh))
        len_crop_dwh = np.insert(crop_dim_wh, 0, np.linalg.norm(crop_dim_wh))
        shrink_factor_dwh = len_full_dwh/len_crop_dwh
        FOV_diagonal_full_rad = FOV_diagonal_full_deg*np.pi/180
        newFOV_rad_dwh = 2*np.arctan((len_full_dwh/len_full_dwh[0] * np.tan(FOV_diagonal_full_rad/2)) / shrink_factor_dwh)
        newFOV_deg_dwh = newFOV_rad_dwh*180/np.pi
        return(newFOV_deg_dwh)

    #Get the main frame image from the camera
    def get_image(self):
        return(self.picam2.capture_array())

    #Get the color mask of the camera's current view
    def get_color_mask(self):
        #Get the image from the camera
        image = self.get_image()
        mask = self.color_filter.get_color_filter_mask(image)
        #Return the resulting mask
        return(mask)
    
    #Change the color filter range
    def set_color_filter(self, hue, precision):
        self.color_filter.set_filter(hue, precision)

    #get the FOV of the camera in degrees
    def get_FOV_deg(self, type_dwh):
        type_dwh = type_dwh.upper()
        if(type_dwh == 'D'):
            return(self.FOV_diagonal_deg)
        elif(type_dwh == 'W'):
            return(self.FOV_width_deg)
        elif(type_dwh == 'H'):
            return(self.FOV_height_deg)
        else:
            raise Exception("Invald dwh type!")

    #get the FOV of the camera in radians
    def get_FOV_rad(self, type_dwh):
        FOV = self.get_FOV_deg(type_dwh)
        return(FOV*np.pi/180)

    #get the maximum pixel dimension in a given direction: (D)iagonal, (W)idth, (H)eight
    def get_max_dimension_pxl(self, type_dwh):
        type_dwh = type_dwh.upper()
        if(type_dwh == 'D'):
            return(np.sqrt(self.width**2 + self.height**2))
        elif(type_dwh == 'W'):
            return(self.width)
        elif(type_dwh == 'H'):
            return(self.height)
        else:
            raise Exception("Invald dwh type!")

    #Get the calibrated main frame image from the camera
    def get_calibrated_image(self, alpha=0.0):
        #Check to see if the camera has been calibrated
        if(not self.isCalibrated):
            raise Exception("Error! Trying to get calibrated image before camera could be properly calibrated!")
        #get the image from the camera alongside its dimensions
        uncalibrated_image = self.picam2.capture_array()
        h, w = uncalibrated_image.shape[0:2]
        #compute the new optimal camera matrix to better undistort the image
        new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(self.calibration_data["camera_matrix"], self.calibration_data["distortion_coefficients"], imageSize=(w,h), alpha=alpha, newImgSize=(w,h))
        new_image = cv2.undistort(uncalibrated_image, self.calibration_data["camera_matrix"], self.calibration_data["distortion_coefficients"], dst=None, newCameraMatrix=new_camera_matrix)
        # crop the image
        print(roi)
        #x, y, w, h = roi
        #new_image = new_image[y:y+h, x:x+w]
        return(new_image)

    #Get the uncalibrated main frame image from the camera
    def get_uncalibrated_image(self):
        return(self.picam2.capture_array())

    #destructor for camera class
    def __del__(self):
        #recommended to close pi-camera from post: https://stackoverflow.com/questions/76548060/access-picamera2-within-a-surrounding-class-on-raspberry-pi
        self.picam2.close()