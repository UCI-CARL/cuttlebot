# https://pyimagesearch.com/2015/09/14/ball-tracking-with-opencv/

# import the necessary packages
from collections import deque
# from imutils.video import VideoStream
import numpy as np
import argparse
import cv2
# import imutils
import time
from picamera2 import Picamera2
import helpers

BLUE_HSV_LOWER_LIMT = np.uint8([100, 100, 100])
BLUE_HSV_UPPER_LIMIT = np.uint8([140, 255, 255])

# Start up rpi camera
camera = Picamera2()
# camera.resolution = (640, 480)
# camera.framerate = 30
camera.preview_configuration.main.size = (640, 480)
camera.preview_configuration.main.format = "RGB888"
camera.preview_configuration.align()
camera.configure("preview")
camera.start()

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video",
	help="path to the (optional) video file")
ap.add_argument("-b", "--buffer", type=int, default=64,
	help="max buffer size")
args = vars(ap.parse_args())

# define the lower and upper boundaries of the "green"
# ball in the HSV color space, then initialize the
# list of tracked points
blue_hls_lower_limit = np.uint8([100, 100, 100])
blue_hls_upper_limit = np.uint8([140, 255, 255])

pts = deque(maxlen=args["buffer"])
# if a video path was not supplied, grab the reference
# to the webcam
# if args.get("video", True):
# 	vs = VideoStream(src=0).start()
# otherwise, grab a reference to the video file
# else:
# vs = cv2.VideoCapture(args["video"])
# allow the camera or video file to warm up
time.sleep(2.0)

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
    mask = helpers.create_mask(frame_hsv, BLUE_HSV_LOWER_LIMT, BLUE_HSV_UPPER_LIMIT)

    # find contours in the mask and initialize the current
	# (x, y) center of the ball
    contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = helpers.grab_contours(contours)
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
            cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
            cv2.circle(frame, centroid, 5, (0, 0, 255), -1) # draw the centroid

    # update the points queue
    pts.appendleft(centroid)

    # loop over the set of tracked points
    for i in range(1, len(pts)):
		# if either of the tracked points are None, ignore them
        if pts[i - 1] is None or pts[i] is None:
            continue
        # otherwise, compute the thickness of the line and
        # draw the connecting lines
        thickness = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
        cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)
        
    # show the frame to our screen
    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1) & 0xFF
    # if the 'q' key is pressed, stop the loop
    if key == 27:
        break

# close all windows
cv2.destroyAllWindows()