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

def compareLines(line1, line2, tolerance):

	attempted = 0.0
	failed = 0.0

	if len(line1) >= len(line2):
		for a in line1:
			attempted += 1.0
			failed += 1.0
			for b in line2:
				if (a + tolerance >= b - tolerance or a - tolerance <= b + tolerance):
					failed -= 1.0
					break
	else:
		for a in line2:
			attempted += 1.0
			failed += 1.0
			for b in line1:
				if (a + tolerance >= b - tolerance or a - tolerance <= b + tolerance):
					failed -= 1.0
					break

	if attempted == 0.0:
		return 1.0

	return 1.0 - failed/attempted

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
mask = cv2.Canny(blurred, 100, 190) #auto_canny(blurred)

# find contours and init the center of the ball
cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
	cv2.CHAIN_APPROX_SIMPLE)[-2]
centroid = None

for c in cnts:
    ((x,y), radius) = cv2.minEnclosingCircle(c)
    M = cv2.moments(c)
    if M["m00"] != 0:
	centroid =(int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
    else: 
	centroid =(0,0)

    # draw circle and centroid, update tracked pts
    cv2.circle(frame, (int(x), int(y)), int(radius),
        (0, 255, 255), 2)
    cv2.circle(frame, centroid, 5, (0, 0, 255), -1)
while(cal):
	cv2.imshow("Frame", mask)

	key = cv2.waitKey(1) & 0xFF

	if key == ord("q"):
		#cv2.destroyAllWindows()
		cal = 0
		print 'Calibration complete'

if len(cnts) == 2:	
	dot1 = cnts[0];
	dot2 = cnts[1];

	((x1,y1), radius1) = cv2.minEnclosingCircle(dot1)
	M1 = cv2.moments(dot1)
	if M1["m00"] != 0:
		dot1_coords =(int(M1["m10"] / M1["m00"]), int(M1["m01"] / M1["m00"]))
	else: 
		dot1_coords =np.matrix([0,0])
	

	((x2,y2), radius2) = cv2.minEnclosingCircle(dot2)
	M2 = cv2.moments(dot2)
	if M2["m00"] != 0:
		dot2_coords =(int(M2["m10"] / M2["m00"]), int(M2["m01"] / M2["m00"]))
	else: 
		dot2_coords =np.matrix([0,0])
	
	scalingConstant = CALIBRATION_DISTANCE/np.hypot(dot2_coords[0] - dot1_coords[0], dot2_coords[1] - dot1_coords[1])

	disp = (dot2_coords[0]-dot1_coords[0],dot2_coords[1] - dot1_coords[1])
	unit = (disp[0]/np.hypot(disp[0], disp[1]),disp[1]/np.hypot(disp[0], disp[1]))

	angle = 0

	if abs(unit[0]) >= abs(unit[1]):
		if unit[0] < 0:
			unit =(unit[0] * -1, unit[1]* -1)
		angle = np.arctan2(1.0 - unit[0], 0.0 - unit[1])
	else:
		if unit[1] < 0:
			unit = (unit[0] * -1, unit[1] * -1)
		angle = np.arctan2(1.0 - unit[1], 0.0 - unit[0])
	rotationMatrix = np.array([[np.cos(angle), -np.sin(angle)],[np.sin(angle), np.cos(angle)]])

else:
	print("Could not find both calibration dots")
	cal = 0
	sys.exit()

#print 'ROTATION:', rotationMatrix
cv2.destroyAllWindows()
time.sleep(1)

binSize = int(1.0/scalingConstant)

history = {}
firstTry = True
threshold = 0.7
tolerance = binSize

try:
	while(go):
		raw2 = PiRGBArray(camera)
		camera.capture(raw2, format="bgr")
		frame2 = raw2.array
	#	print 'captured'
		#cv2.imshow("suh", frame2)
		#cv2.waitKey(0)
	        
		# grab frame
		grey = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)
		blurred = cv2.GaussianBlur(grey, (3,3), 0)
		#thresh1 = cv2.threshold(blurred, 65, 255, cv2.THRESH_BINARY)[1]
		#thresh1 = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY, 111,1)
		#thresh1 = cv2.bitwise_not(thresh1, thresh1)

		thresh1 = cv2.Canny(blurred, 100, 190) #auto_canny(blurred)
		
		#cv2.imshow("thresh", thresh1)
		#cv2.waitKey(0)

		# Process the contours: find the slots
		contours = cv2.findContours(thresh1.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		contours = contours[1] # the second tuple is correct for CV3

	#	print 'contour length:', len(contours)
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
		
				dots = np.vstack((dots,np.dot(np.matrix([cX, cY]),rotationMatrix)))

		dots = dots.tolist()

		indices = np.digitize([dot[0] for dot in dots], [i*binSize for i in range(1280/binSize)])

		lines = {}
		for i in range(1280/binSize):
			lines[i] = []

		for i in range(len(dots)):
			lines[indices[i]] = dots[i][1]

		if firstTry:
		
			history = lines
		else:
			# compare
	
			n = 1280/binSize

			while(True):
				
				percentMatches = []
				for i in range(n):

					percentMatches.append(compareLines(lines[i], history[i-n], tolerance))
		
				averageMatch = sum(percentMatches)/float(len(percentMatches))

				if (averageMatch >= threshold):
					break

				n = n - 1

				if n == 0:
					break

			for i in range(1280/binSize-n):
				history[len(history)] = lines[i-(1280/binSize-n)]

		firstTry = False

		cv2.drawContours(frame, [c], -1, (0, 0, 255), 2)
		cv2.circle(frame, (cX, cY), 3, (0,255, 0), -1)
	
		cv2.imshow("Frame", frame)
	    	key = cv2.waitKey(1) & 0xFF
	
		if key == ord("q"):
			go = 0


except Exception as e:
	print e
	go = 0

cv2.destroyAllWindows()

print history