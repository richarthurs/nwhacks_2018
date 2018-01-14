from collections import deque
import numpy as np
import argparse
import imutils
import cv2

from picamera.array import PiRGBArray
from picamera import PiCamera
import time

ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video",
	help="Path to the optional video file")
ap.add_argument("-b", "--buffer", type=int, default=64,
	help="Max buffer size")
args = vars(ap.parse_args())

# define lower and upper bounds of green in HSV, init list of tracked points
greenLower = (29, 86, 6)
greenUpper = (64, 255, 255)
pts = deque(maxlen=args["buffer"])

# get a path to the pi camera
if not args.get("video", False):
	camera = PiCamera()
	camera.resolution = (640, 480)
	camera.framerate = 32
	rawCapture = PiRGBArray(camera, size=(640, 480))
	time.sleep(0.1)

# otherwise, snag a ref to the video file
else: camera = cv2.VideoCapture(args["video"])

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port = True):
	grabbed = 1
	
	img = frame.array

	# HSV format the frame
	# blurred = cv2.GaussianBlur(frame, (11,11), 0)
	hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

	# make a mask for green, remove small blobs with dialation and erosion
	mask = cv2.inRange(hsv, greenLower, greenUpper)
	mask = cv2.erode(mask, None, iterations = 2)
	mask = cv2.dilate(mask, None, iterations=2)

	# find contours and init the center of the ball
	cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE)[-2]
	center = None

	# only proceed if at least one contour was found
	if len(cnts) > 0:
		# find largest contour in mask, then compute enclosing circle and centroid
		c = max(cnts, key=cv2.contourArea)
		((x,y), radius) = cv2.minEnclosingCircle(c)
		M = cv2.moments(c)
		center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

		# only proceed if thr radius meets a min size
		if radius > 10:
			# draw circle and centroid, update tracked pts
			cv2.circle(img, (int(x), int(y)), int(radius),
				(0, 255, 255), 2)
			cv2.circle(img, center, 5, (0, 0, 255), -1)
		print center 
	pts. appendleft(center)

	for i in xrange(1, len(pts)):
		# if either of the points are None, ignore em
		if pts[i - 1] is None or pts[i] is None:
			continue

		# otherwise, compute thickness of line and draw it
		thickness = int(np.sqrt(args["buffer"] / float(i+1)) * 2.5)
		cv2.line(img, pts[i - 1], pts[i], (0, 0, 255), thickness)

	# show image
	cv2.imshow("Frame", img)
	cv2.imshow("Mask", mask)
	key = cv2.waitKey(1) & 0xFF
	
	# clear the stream
	rawCapture.truncate(0)

	# stop loop if q is pressed
	if key == ord("q"):
		break

# cleanup cam and close windows
cv2.destroyAllWindows()

