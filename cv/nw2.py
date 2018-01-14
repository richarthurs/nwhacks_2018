from cv_utils import piStream
from cv_utils import FPS
from picamera.array import PiRGBArray
from picamera import PiCamera
import argparse
import time
import cv2

# This script incorporates OpenCV usage from pyimagesearch.com to find the centroid
# of a green object, and draw a circle around it using the Pi's built in camera.
# http://richarthurs.com/2017/08/20/getting-started-with-opencv-and-raspberry-pi/

camera = PiCamera()
raw = PiRGBArray(camera)
time.sleep(0.1)
go = 1

try:
	while(go):
		cv2.waitKey(0)
		camera.capture(raw, format="bgr")
		frame = raw.array
	
	        # grab frame
		grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		blurred = cv2.GaussianBlur(grey, (1,1), 0)
		thresh1 = cv2.threshold(blurred, 35, 255, cv2.THRESH_BINARY)[1]
		#thresh1 = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY, 115,1)
		thresh1 = cv2.bitwise_not(thresh1, thresh1)
		
		# Process the contours: find the slots
		contours = cv2.findContours(thresh1.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		contours = contours[1] # the second tuple is correct for CV3
	
		if (len(contours) > 0):
			for c in contours:
				# find the centre
				M = cv2.moments(c)
				if M["m00"] != 0:
					cX = int(M["m10"] / M["m00"])
					cY = int(M["m01"] / M["m00"])
				else: 
					cX = 0
					cY = 0
	
				print [cX, cY]
				
				# draw the contour and centre in the image
				#cv2.drawContours(frame, [c], -1, (0, 0, 255), 2)
				cv2.circle(frame, (cX, cY), 3, (0,255, 0), -1)
	
	        cv2.imshow("Frame", frame)
	    	key = cv2.waitKey(1) & 0xFF
	
		if key == ord("q"):
			go = 0

	
except Exception as e:
	print e
	go = 0



cv2.destroyAllWindows()


