from cv_utils import piStream
from cv_utils import FPS
from picamera.array import PiRGBArray
from picamera import PiCamera
import argparse
import time
import cv2
import numpy as np
import sys

# This script incorporates OpenCV usage from pyimagesearch.com to find the centroid
# of a green object, and draw a circle around it using the Pi's built in camera.
# http://richarthurs.com/2017/08/20/getting-started-with-opencv-and-raspberry-pi/
def auto_canny(image, sigma=0.33):
	# compute the median of the single channel pixel intensities
	v = np.median(image)
 
	# apply automatic Canny edge detection using the computed median
	lower = int(max(0, (1.0 - sigma) * v))
	upper = int(min(255, (1.0 + sigma) * v))
	edged = cv2.Canny(image, lower, upper)
 
	# return the edged image
	return edged

camera = PiCamera()
camera.resolution = (1280,720)
raw = PiRGBArray(camera)
time.sleep(0.1)
go = 1
dots = np.zeros([1,2])

redLower = (20, 20, 100)
redUpper = (70, 70, 255)

CALIBRATION_DISTANCE = 10.0
scalingConstant = 1.0
rotationMatrix = []

# calibrate
cal = 1

#frame = vs.read()
camera.capture(raw, format="bgr")
frame = raw.array
hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

# make a mask for green, remove small blobs with dialation and erosion
#mask = cv2.inRange(hsv, redLower, redUpper)
#mask = cv2.erode(mask, None, iterations = 2)
#mask = cv2.dilate(mask, None, iterations=2)

# Canny detection
gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
blurred = cv2.GaussianBlur(gray, (3, 3), 0)
mask = cv2.Canny(blurred, 225, 250) #auto_canny(blurred)

# find contours and init the center of the ball
cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
	cv2.CHAIN_APPROX_SIMPLE)[-2]
centroid = None

for c in cnts:
    ((x,y), radius) = cv2.minEnclosingCircle(c)
    M = cv2.moments(c)
    centroid = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

    # draw circle and centroid, update tracked pts
    cv2.circle(frame, (int(x), int(y)), int(radius),
        (0, 255, 255), 2)
    cv2.circle(frame, centroid, 5, (0, 0, 255), -1)
while(cal):
	cv2.imshow("Frame", frame)

	key = cv2.waitKey(1) & 0xFF

	if key == ord("q"):
		cv2.destroyAllWindows()
		cal = 0

if len(cnts) == 2:	
	dot1 = cnts[0];
	dot2 = cnts[1];

	((x1,y1), radius1) = cv2.minEnclosingCircle(dot1)
	M1 = cv2.moments(dot1)
	dot1_coords =.np.matrix([int(M1["m10"] / M1["m00"]), int(M1["m01"] / M1["m00"])])

	((x2,y2), radius2) = cv2.minEnclosingCircle(dot2)
	M2 = cv2.moments(dot2)
	dot2_coords = np.matrix([int(M2["m10"] / M2["m00"]), int(M2["m01"] / M2["m00"])])
	
	scalingConstant = CALIBRATION_DISTANCE/np.hypot(dot2_coords[0] - dot1_coords[0], dot2_coords[1] - dot1_coords[1])

	disp = np.matrix(dot2_coords - dot1_coords)
	unit = np.matrix(disp/np.hypot(disp[0], disp[1]))

	angle = 0
	if abs(unit[0]) >= abs(unit[1]):
		if unit[0] < 0:
			unit = np.matrix(unit * -1)
		angle = np.arctan2(1.0 - unit[0], 0.0 - unit[1])
	else:
		if unit[1] < 0:
			unit = np.matrix(unit * -1)
		angle = np.arctan2(1.0 - unit[1], 0.0 - unit[0])
	rotationMatrix = np.matrix([[np.cos(angle), -np.sin(angle)],[np.sin(angle), np.cos(angle)]])

else:
	print("Could not find both calibration dots")
	cal = 0
	sys.exit()

try:
	while(go):
		cv2.waitKey(0)
		camera.capture(raw, format="bgr")
		frame = raw.array
	
	        # grab frame
		grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		blurred = cv2.GaussianBlur(grey, (3,3), 0)
		thresh1 = cv2.threshold(blurred, 65, 255, cv2.THRESH_BINARY)[1]
		#thresh1 = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY, 111,1)
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
				dots = np.vstack((dots,np.multiply(rotationMatrix,np.matrix[cX, cY])))
				
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

print "dots: --------------"
dots = dots[~np.all(dots == 0, axis=1)] #https://stackoverflow.com/questions/11188364/remove-zero-lines-2-d-numpy-array
print dots
cv2.destroyAllWindows()


