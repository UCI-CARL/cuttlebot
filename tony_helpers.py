import cv2
import RPi.GPIO as GPIO
import time
from picamera2 import Picamera2
import libcamera

def set_servo_angle(servo_pin, angle):
	# Must initialize, start and stop pwm every time when setting the servo angle for some reason
	pwm = GPIO.PWM(servo_pin, 50)
	pwm.start(0)
	dutyCycle = (0.01056 * angle + 0.5) / 20.0 * 100
	pwm.ChangeDutyCycle(dutyCycle)
	time.sleep(0.1)
	pwm.stop()

def create_mask(frame_hsv, hsv_lower_limit, hsv_upper_limit):
	mask = cv2.inRange(frame_hsv, hsv_lower_limit, hsv_upper_limit)
	mask = cv2.erode(mask, None, iterations=2)
	mask = cv2.dilate(mask, None, iterations=2)
	
	return mask

def start_camera():
	camera = Picamera2()
	# camera.resolution = (640, 480)
	# camera.framerate = 30
	config = camera.create_preview_configuration(
		main = {
			"size": (640, 480),
			"format": "XRGB8888" # Set the correct format to avoid weird colors
			},
		transform = libcamera.Transform(hflip=0, vflip=1)
	)
	camera.configure(config)
	camera.start()
	time.sleep(1) # Wait for the camera to warm up

	return camera