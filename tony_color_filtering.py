# https://maker.pro/raspberry-pi/tutorial/how-to-create-object-detection-with-opencv

import cv2
import numpy as np
from picamera2 import Picamera2

def nothing(x):
    pass
 
# cv2.namedWindow("Trackbars")

# cv2.createTrackbar("B", "Trackbars", 0, 255, nothing)
# cv2.createTrackbar("G", "Trackbars", 0, 255, nothing)
# cv2.createTrackbar("R", "Trackbars", 0, 255, nothing)

# Start up rpi camera
camera = Picamera2()
# camera.main.size = (640, 480)
# camera.framerate = 30
# camera.main.format = "RGB888"
camera.preview_configuration.main.size = (640, 480)
camera.preview_configuration.main.format = "RGB888"
camera.preview_configuration.align()
camera.configure("preview")
camera.start()

BLUE_HSV_LOWER_LIMT = np.uint8([100, 100, 100])
BLUE_HSV_UPPER_LIMIT = np.uint8([140, 255, 255])

def color_red_mask(frame_hsv):
	red_hsv_lower_limit_low_angle = np.uint8([0,150, 150])
	red_hsv_upper_limit_low_angle = np.uint8([10, 255, 255])
	red_hsv_lower_limit_high_angle = np.uint8([170, 150, 150])
	red_hsv_upper_limit_high_angle = np.uint8([179, 255, 255])

    #Get the mask (boolean matrix) for pixels that are in the specified range
	red_low_angle_mask = cv2.inRange(frame_hsv, red_hsv_lower_limit_low_angle, red_hsv_upper_limit_low_angle)
	red_high_angle_mask = cv2.inRange(frame_hsv, red_hsv_lower_limit_high_angle, red_hsv_upper_limit_high_angle)

	return cv2.bitwise_or(red_low_angle_mask, red_high_angle_mask)

def create_mask(frame_hsv, hsv_lower_limit, hsv_upper_limit):
	mask = cv2.inRange(frame_hsv, hsv_lower_limit, hsv_upper_limit)
	mask = cv2.erode(mask, None, iterations=2)
	mask = cv2.dilate(mask, None, iterations=2)
	
	return mask

while True:
	frame = camera.capture_array()
	blurred_frame = cv2.GaussianBlur(frame, (11, 11), 0)

	blurred_frame_hsv = cv2.cvtColor(blurred_frame, cv2.COLOR_BGR2HSV)
	# B = cv2.getTrackbarPos("B", "Trackbars")
	# G = cv2.getTrackbarPos("G", "Trackbars")
	# R = cv2.getTrackbarPos("R", "Trackbars")

    # Find out the lower and upper limit of the color in HSV
	# color = np.uint8([[[B, G, R]]])
	
	# Adjust the threshold of the HSV image for a range of each selected color
	blue_mask = create_mask(blurred_frame_hsv, BLUE_HSV_LOWER_LIMT, BLUE_HSV_UPPER_LIMIT)
	
	result = cv2.bitwise_and(frame, frame, mask=blue_mask) # Apply the mask

	cv2.imshow("frame", frame)
	# # cv2.imshow("mask", mask)
	cv2.imshow("result", result)
	
	if cv2.waitKey(1) == 27:
		break

cv2.destroyAllWindows()

