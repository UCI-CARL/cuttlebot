#import needed libraries
import cv2
import picamera2
import libcamera
import numpy as np
import RPi.GPIO as GPIO
import time

#set GPIO numbering mode (left-right;top-down numbering starting from 1)
GPIO.setmode(GPIO.BOARD)

#Set pin 11 as an output pin
bot_pin = 13
top_pin = 11
GPIO.setup(bot_pin, GPIO.OUT)
GPIO.setup(top_pin, GPIO.OUT)

#set PWM to 50Hz (servo expects 50Hz PWM to read from: 20 ms per pulse)
bot_servo = GPIO.PWM(bot_pin, 50)
top_servo = GPIO.PWM(top_pin, 50)

#begin the servo motors in off state
bot_servo.start(0)
top_servo.start(0)
time.sleep(1)
bot_servo.ChangeDutyCycle(7.5)
top_servo.ChangeDutyCycle(7.5)
time.sleep(1)
bot_servo.ChangeDutyCycle(0)
top_servo.ChangeDutyCycle(0)

#Instantiate the camera objejct
picam = picamera2.Picamera2()
time.sleep(1)

#set configuration for output streams
config = picam.create_preview_configuration(
    #main stream
    main = {
        "size": (640, 480), #width x height
        "format": "XRGB8888" #8-bit [B, G, R, 255]
    },
    
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

#create a function that gets the centerpoint of a masked image
def get_object_center_point(mask):
    #using reference to get center point: https://answers.opencv.org/question/204175/how-to-get-boundry-and-center-information-of-a-mask/
    # Find contours:
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    # Draw contours:
    try:
        # Calculate image moments of the detected contour
        c = max(contours, key = cv2.contourArea)
        x,y,w,h = cv2.boundingRect(c)
        relative_center_point = ((x+w/2) - mask.shape[1]/2, (y+h/2) - mask.shape[0]/2)
        return(np.array(relative_center_point))
    except:
        return(None)

hls_lower_limit_lowAngleRED = np.uint8([0,135, 135])
hls_upper_limit_lowAngleRED = np.uint8([15, 255, 255])
low_angle_filter_range = (hls_lower_limit_lowAngleRED, hls_upper_limit_lowAngleRED)

hls_lower_limit_highAngleRED = np.uint8([165, 135, 135])
hls_upper_limit_highAngleRED = np.uint8([180, 255, 255])
high_angle_filter_range = (hls_lower_limit_highAngleRED, hls_upper_limit_highAngleRED)

#(bot, top)
PWM_vec = (7.5, 7.5)
prev_error = np.array([0.0, 0.0])
total_error = np.array([0.0, 0.0])
def changePWM(point):
    global PWM_vec
    global prev_error
    global total_error
    global bot_servo
    global top_servo
    print("==========")
    print(f"point {point}")
    if(point is None):
        return
    k_p = 0.5
    k_i = 0.25
    k_d = 0.25
    #compute the errors (from reference point of (0,0), the point is the error)
    error = point*(np.abs(point) >= 50) - 50
    print(f"error {error}")
    #compute the P value
    P_vec = k_p*(error)
    #compute the I value
    total_error += error
    I_vec = np.array([0.0,0.0]) #k_i*(total_error)
    #compute the D value
    D_vec = np.array([0.0,0.0]) #k_d*(error - prev_error)
    prev_error = error

    #Get the total sum of the PID
    print(f"P_vec {P_vec}")
    print(f"I_vec {I_vec}")
    print(f"D_vec {D_vec}")
    PID_vec = P_vec + I_vec + D_vec
    PID_vec[0] *= -1
    print(f"PID_vec {PID_vec}")

    #change the PWM values (based on scale factor for error->PWM)
    PWM_vec += 0.0001*(PID_vec)
    #ensure the values are in range of the PWM
    PWM_vec = np.clip(PWM_vec, 2.5, 12.5)
    print(f"PWM_vec {PWM_vec}")

    #set the PWM
    bot_servo.ChangeDutyCycle(PWM_vec[0])
    top_servo.ChangeDutyCycle(PWM_vec[1])
    time.sleep(0.015)
    bot_servo.ChangeDutyCycle(0)
    top_servo.ChangeDutyCycle(0)

while(1):
    #Get the image frame
    main_frame = picam.capture_array()

    #blur the image to remove any noise (small colored objects in the background)
    #filter of size 12 used as stated here: http://www.roborealm.com/tutorial/color_object_tracking_2/slide010.php
    blurred_frame = cv2.blur(main_frame, (12,12))

    #Convert the image into an HSV image
    HSV_frame = cv2.cvtColor(blurred_frame, cv2.COLOR_BGR2HSV)

    #Get the mask (boolean matrix) for pixels that are in the specified range
    lowAngle_mask = cv2.inRange(HSV_frame, *low_angle_filter_range)
    highAngle_mask = cv2.inRange(HSV_frame, *high_angle_filter_range)
    
    #create one mask using bitwise or
    mask = cv2.bitwise_or(lowAngle_mask, highAngle_mask)
    center_point = get_object_center_point(mask)

    #apply the mask to the image. Can think of the mask as a window to view the object
    masked_image = cv2.bitwise_and(main_frame, main_frame, mask=mask)
    #print(center_point)
    changePWM(center_point)

    #Print the image
    cv2.imshow("Red Filter Image Result", masked_image)
    #Wait for key input (ESC) to quit the loop
    if(cv2.waitKey(1) == 27):
        break

#Clean up objects
cv2.destroyAllWindows()
picam.stop()
