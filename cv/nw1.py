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

vs = piStream((1000, 620)).start()
time.sleep(2.0)
fps = FPS().start()

go = 1
while go:
	try:
	        # grab frame
	        frame = vs.read()
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
	
		#fps.update()
	
		if key == ord("q"):
			go = 0
	               	break
		
	
	#	#hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
	
		# make a mask for green, remove small blobs with dialation and erosion
		# mask = cv2.inRange(hsv, greenLower, greenUpper)
		# mask = cv2.erode(mask, None, iterations = 2)
		# mask = cv2.dilate(mask, None, iterations=2)
	
		# find contours and init the center of the ball
		# cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
			# cv2.CHAIN_APPROX_SIMPLE)[-2]
		#centroid = None
	
		# only proceed if at least one contour was found
		# if len(cnts) > 0:
			# find largest contour in mask, then compute enclosing circle and centroid
			# c = max(cnts, key=cv2.contourArea)
			# ((x,y), radius) = cv2.minEnclosingCircle(c)
			# M = cv2.moments(c)
			# centroid = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
	
			# only proceed if thr radius meets a min size
			# if radius > 10:
				# draw circle and centroid, update tracked pts
			#	cv2.circle(frame, (int(x), int(y)), int(radius),
			#		(0, 255, 255), 2)
			#	cv2.circle(frame, centroid, 5, (0, 0, 255), -1)
			#print centroid 
	except Exception as e:
		print e
		go = 0
 

fps.stop()
print "Elapsed time: ", fps.elapsed()
print "FPS: ", fps.fps()

cv2.destroyAllWindows()
vs.stop()

