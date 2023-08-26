# https://pyimagesearch.com/2016/05/09/opencv-rpi-gpio-and-gpio-zero-on-the-raspberry-pi/

# import the necessary packages
# from imutils.video import VideoStream
import numpy as np
import cv2
import imutils
# from imutils.video import VideoStream
import RPi.GPIO as GPIO
import tony_helpers
import time

# Color filtering
BLUE_HSV_LOWER_LIMT = np.uint8([100, 100, 100])
BLUE_HSV_UPPER_LIMIT = np.uint8([140, 255, 255])

# GPIOs
LED_PIN = 21
PAN_SERVO_PIN = 26
TILT_SERVO_PIN = 27
PAN_INITIAL_DUTY_CYCLE = 7 # ((2.4 - 0.5) / 180 * 90 + 0.5) / 20 * 100
TILT_INITIAL_DUTY_CYCLE = 5 # ((2.4 - 0.5) / 180 * 45 + 0.5) / 20 * 100

CAMERA_CENTER_RANGE_X = (290, 350)
CAMERA_CENTER_RANGE_Y = (220, 260)

# position servos to present object at center of the frame
def servo_tracking(x, y):
    print("x:", x, "y:", y)
    global pan_angle
    global tilt_angle
    if (x < CAMERA_CENTER_RANGE_X[0]):
        pan_angle -= 10
        if pan_angle < 0:
            pan_angle = 0
        tony_helpers.set_servo_angle(PAN_SERVO_PIN, pan_angle)
    if (x > CAMERA_CENTER_RANGE_X[1]):
        pan_angle += 10
        if pan_angle > 180:
            pan_angle = 180
        tony_helpers.set_servo_angle(PAN_SERVO_PIN, pan_angle)
    if (y < CAMERA_CENTER_RANGE_Y[0]):
        tilt_angle -= 10
        if tilt_angle < 0:
            tilt_angle = 0
        tony_helpers.set_servo_angle(TILT_SERVO_PIN, tilt_angle)
    if (y > CAMERA_CENTER_RANGE_Y[1]):
        tilt_angle += 10
        if tilt_angle > 180:
            tilt_angle = 180
        tony_helpers.set_servo_angle(TILT_SERVO_PIN, tilt_angle)

pan_angle = 90
tilt_angle = 45

# Set up GPIO pins
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(PAN_SERVO_PIN, GPIO.OUT)
GPIO.setup(TILT_SERVO_PIN, GPIO.OUT)

# Start up rpi camera
camera = tony_helpers.start_camera()
# Move Pan-tilt unit to the initial positions
tony_helpers.set_servo_angle(PAN_SERVO_PIN, 90)
tony_helpers.set_servo_angle(TILT_SERVO_PIN, 45)

# keep looping
while True:
	# grab the current frame
	frame = camera.capture_array()
	# handle the frame from VideoCapture or VideoStream
	# frame = frame[1] if args.get("video", False) else frame
	# if we are viewing a video and we did not grab a frame,
	# then we have reached the end of the video
	if frame is None:
		break
	# resize the frame, blur it, and convert it to the HSV
	# color space
	# frame = imutils.resize(frame, width=600)
	blurred_frame_hsv = cv2.GaussianBlur(frame, (11, 11), 0)
	frame_hsv = cv2.cvtColor(blurred_frame_hsv, cv2.COLOR_BGR2HSV)

	# construct a mask for the color "green", then perform
	# a series of dilations and erosions to remove any small
	# blobs left in the mask
	mask = tony_helpers.create_mask(frame_hsv, BLUE_HSV_LOWER_LIMT, BLUE_HSV_UPPER_LIMIT)

	# find contours in the mask and initialize the current
	# (x, y) center of the ball
	contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	contours = imutils.grab_contours(contours) # Ensure opencv compatibility
	centroid = None
	
	# only proceed if at least one contour was found
	if len(contours) > 0:
		# find the largest contour in the mask, then use
		# it to compute the minimum enclosing circle and
		# centroid
		c = max(contours, key=cv2.contourArea)
		((x, y), radius) = cv2.minEnclosingCircle(c) # Get the center and radius of the enclosing circle
		M = cv2.moments(c)
		centroid = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

		# only proceed if the radius meets a minimum size
		if radius > 10:
			# draw the circle and centroid on the frame,
			# then update the list of tracked points
			cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2) # Draw the enclosing circle
			cv2.circle(frame, centroid, 5, (0, 0, 255), -1) # Draw the centroid
			
			servo_tracking(int(x), int(y))

			# # if the led is not already on, raise an alarm and
			# # turn the LED on
			# if not led_on:
			# 	GPIO.output(LED_PIN, GPIO.HIGH)
			# 	led_on = True

	# elif led_on:
	# 	GPIO.output(LED_PIN, GPIO.LOW)
	# 	led_on = False

	# show the frame to our screen
	cv2.imshow("Frame", frame)
	key = cv2.waitKey(1) & 0xFF
	# if the 'q' key is pressed, stop the loop
	if key == 27:
		break

# Cleanup
print("\n [INFO] Exiting Program and cleanup stuff \n")
tony_helpers.set_servo_angle(PAN_SERVO_PIN, 90)
tony_helpers.set_servo_angle(TILT_SERVO_PIN, 45)
GPIO.cleanup()
# close all windows
cv2.destroyAllWindows()