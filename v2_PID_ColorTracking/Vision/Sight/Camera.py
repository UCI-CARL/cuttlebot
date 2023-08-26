#import needed libraries
import picamera2
import libcamera
import time
from Vision.Sight.Filters.ColorFilter import ColorFilter

class Camera():
    def __init__(self):
        #Instantiate the pi-camera object
        self.picam2 = picamera2.Picamera2()
        #set configuration for output streams
        config = self.picam2.create_preview_configuration(
            #main stream
            main = {
                "size": (320, 240), #width x height
                "format": "XRGB8888" #8-bit [B, G, R, 255]
            },
            
            #video controls
            #controls = {
            #    "FrameDurationLimits": (333333, 333333) #microseconds boundary
            #},

            #other parameters
            transform = libcamera.Transform(hflip=True, vflip=False), #params set so that the positive and negative axis are in right direction
            #queue = True,
            display = None #None, "main", or "lores"
        )
        #apply the configuration to the camera object
        self.picam2.configure(config)
        #start the camera
        self.picam2.start()
        #wait a second after starting to allow camera to fully start up
        time.sleep(1)

        #Instantiate the color filter object
        self.color_filter = ColorFilter()
    
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