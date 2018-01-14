from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2

# init camera
camera = PiCamera()
rawCapture = PiRGBArray(camera)

# allow for warmup time
time.sleep(0.1)

# grab an image
camera.capture(rawCapture, format="bgr")
image = rawCapture.array

# show image and wait for key
cv2.imshow("Image", image)
cv2.waitKey(0)
