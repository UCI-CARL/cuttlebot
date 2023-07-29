#track an object of a given color
#referencing https://maker.pro/raspberry-pi/tutorial/how-to-create-object-detection-with-opencv
import cv2
import picamera2
import libcamera
import time
import numpy as np
import matplotlib.pyplot as plt

#Instantiate the camera objejct
picam = picamera2.Picamera2()

#set configuration to enable the low resolution (lores) and raw streams (main stream enabled by default)
#comment out the increase framerate
#picam.preview_configuration.enable_lores()
#picam.preview_configuration.enable_raw()

#set configuration for output streams
config = picam.create_preview_configuration(
    #main stream
    main = {
        "size": (640, 480), #width x height
        "format": "XRGB8888" #8-bit [B, G, R, 255]
    },
    
    #low resolution stream
    lores = {
        "size": (320, 240), #width x height
        "format": "YUV420"
    },
    
    #raw stream
    #comment out as this applies only to the fisheye lens camera
    #raw = picam.sensor_modes[2],
    
    #video controls
    controls = {
        "FrameDurationLimits": (333333, 333333) #microseconds
    },

    #other parameters
    transform = libcamera.Transform(hflip=True, vflip=True),
    queue = True,
    display = None #None, "main", or "lores"
)

#apply the configuration to the camera object
picam.configure(config)

#start the camera process
picam.start()

#get one image frame (as an np.array) from the main, low res, and raw streams
#we only need the main frame now
#(main_frame, lores_frame, raw_frame), metadata = picam.capture_arrays(["main", "lores", "raw"])
main_frame = picam.capture_array()

#print the 3 different frames
#plt.imshow(cv2.cvtColor(main_frame, cv2.COLOR_BGR2RGB))
#print("main frame")
#plt.show()

#plt.imshow(cv2.cvtColor(lores_frame, cv2.COLOR_BGR2RGB))
#print("low resolution frame")
#plt.show()

#plt.imshow(cv2.cvtColor(raw_frame, cv2.COLOR_BGR2RGB))
#print("raw frame")
#plt.show()

#start pyplot for continuous updating
#DONT USE PYPLOT: Its slow! Use cv2.imshow
#plt.ion()
#plt.show()

#loop for 500 frames to get gamera images
for i in range(500):
    #get one image frame (as an np.array) from the main, low res, and raw streams
    (main_frame, lores_frame, raw_frame), metadata = picam.capture_arrays(["main", "lores", "raw"])
    
    #blur the image to help neglect any small colored objects (may need to blur this more)
    blurred_bgr_image = cv2.GaussianBlur(main_frame, (7,7), 0)
    #plt.imshow(cv2.cvtColor(blurred_bgr_image, cv2.COLOR_BGR2RGB))
    #print("blurred image")
    #plt.show()

    #convert the color image to a HSV image (similar to HSL: https://en.wikipedia.org/wiki/HSL_and_HSV)
    hue_value_saturation_image = cv2.cvtColor(blurred_bgr_image, cv2.COLOR_BGR2HSV)
    #plt.imshow(hue_value_saturation_image)
    #print("HLS image")
    #print(hue_calue_saturation_image)
    #plt.show()

    #set the red filter 0 +/- 20: Need to set 2 bounds since -20 degrees = 160 degrees for HSV values
    hls_lower_limit_lowAngleRED = np.uint8([0,128, 128])
    hls_upper_limit_lowAngleRED = np.uint8([20, 255, 255])

    hls_lower_limit_highAngleRED = np.uint8([160, 100, 100])
    hls_upper_limit_highAngleRED = np.uint8([180, 255, 255])

    #Get the mask (boolean matrix) for pixels that are in the specified range
    lowAngle_mask = cv2.inRange(hue_value_saturation_image, hls_lower_limit_lowAngleRED, hls_upper_limit_lowAngleRED)
    highAngle_mask = cv2.inRange(hue_value_saturation_image, hls_lower_limit_highAngleRED, hls_upper_limit_highAngleRED)

    #create one mask using bitwise or
    mask = cv2.bitwise_or(lowAngle_mask, highAngle_mask)
    #plt.imshow(mask)
    #print("image mask")
    #print(mask)
    #plt.show()

    #apply the mask to the image. Can think of the mask as a window to view the object
    masked_image = cv2.bitwise_and(main_frame, main_frame, mask=mask)
    #Pyplot slow! Using cv2 instead
    #plt.clf()
    #plt.imshow(cv2.cvtColor(masked_image, cv2.COLOR_BGR2RGB))
    #print("masked image")
    #plt.draw()
    #plt.pause(0.001)
    cv2.imshow("Red Filter Image Result", masked_image)
    
picam.stop()