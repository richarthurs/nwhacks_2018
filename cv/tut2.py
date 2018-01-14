from cv_utils import piStream
from cv_utils import FPS
from picamera.array import PiRGBArray
from picamera import PiCamera
import argparse
import time
import cv2
import serial

# This script incorporates OpenCV usage from pyimagesearch.com to find the centroid
# of a green object, and draw a circle around it using the Pi's built in camera.
# http://richarthurs.com/2017/08/20/getting-started-with-opencv-and-raspberry-pi/

# This script extends the 'tut1' script to output centroid locations over the serial port
# Richard Arthurs

#------------ Camera Start -------------------
vs = piStream().start()
time.sleep(2.0)
fps = FPS().start()

#------------ Serial Start -------------------
ser = serial.Serial('/dev/serial0', 115200, timeout = 1)
print ser.name

#------------ CV -----------------------------
# define lower and upper bounds of green in HSV, init list of tracked points
greenLower = (29, 86, 6)
greenUpper = (64, 255, 255)

while 1:
        # grab frame
        frame = vs.read()

	# HSV format the frame
	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

	# make a mask for green, remove small blobs with dialation and erosion
	mask = cv2.inRange(hsv, greenLower, greenUpper)
	mask = cv2.erode(mask, None, iterations = 2)
	mask = cv2.dilate(mask, None, iterations=2)

	# find contours and init the center of the ball
	cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE)[-2]
	centroid = None

	# only proceed if at least one contour was found
	if len(cnts) > 0:
		# find largest contour in mask, then compute enclosing circle and centroid
		c = max(cnts, key=cv2.contourArea)
		((x,y), radius) = cv2.minEnclosingCircle(c)
		M = cv2.moments(c)
		centroid = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
		print centroid[1]
		ser.write(str(centroid[1]))

		# only proceed if thr radius meets a min size
		if radius > 10:
			# draw circle and centroid, update tracked pts
			cv2.circle(frame, (int(x), int(y)), int(radius),
				(0, 255, 255), 2)
			cv2.circle(frame, centroid, 5, (0, 0, 255), -1)


        cv2.imshow("Frame", frame)
        key = cv2.waitKey(1) & 0xFF

        fps.update()

        if key == ord("q"):
                break

fps.stop()
print "Elapsed time: ", fps.elapsed()
print "FPS: ", fps.fps()

cv2.destroyAllWindows()
vs.stop()
ser.close()

