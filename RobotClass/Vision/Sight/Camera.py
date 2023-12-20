#import needed libraries
import picamera2
import libcamera
import numpy as np
import time
from Vision.Sight.Filters.ColorFilter import ColorFilter

class Camera():
    def __init__(self):
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



    #destructor for camera class
    def __del__(self):
        #recommended to close pi-camera from post: https://stackoverflow.com/questions/76548060/access-picamera2-within-a-surrounding-class-on-raspberry-pi
        self.picam2.close()