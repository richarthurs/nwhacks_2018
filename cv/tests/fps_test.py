from piStream import piStream
from FPS import FPS
from picamera.array import PiRGBArray
from picamera import PiCamera
import argparse
import time
import cv2

ap = argparse.ArgumentParser()
ap.add_argument("-n", "--num-frames", type = int, default = 100, help = "Number of frames for FPS test")
ap.add_argument("-d", "--display", type = int, default = 3, help = "Show frames or not")
args = vars(ap.parse_args())

# init the camera!

print "Sampling frames from threaded camera module..."
vs = piStream().start()
time.sleep(2.0)
fps = FPS().start()

while fps._numFrames < args["num_frames"]:
	# grab frame
	frame = vs.read()

	if args["display"] > 0:
		cv2.imshow("Frame", frame)
		key = cv2.waitKey(1) & 0xFF
	
	fps.update()

fps.stop()
print "Elapsed time: ", fps.elapsed()
print "FPS: ", fps.fps()

cv2.destroyAllWindows()
vs.stop()

