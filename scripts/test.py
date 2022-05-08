# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import json
from numpy import array, zeros, isclose
# initialize the camera and grab a reference to the raw camera capture
# Resolution used when checking for motion (low resolution for faster searching)
def loadConfig(fpath):
    return json.load(open(fpath, 'r'))

config = loadConfig('../config/camera.conf.json')


# Resolution used to check for motion in the frame
searchResolution = (config["searchResolution"]["width"], config["searchResolution"]["height"])
saveResolution = (config["saveResolution"]["width"], config["saveResolution"]["height"])

camera = PiCamera()
camera.resolution = searchResolution
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=searchResolution)
# allow the camera to warmup
time.sleep(0.1)
# capture frames from the camera

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    # grab the raw NumPy array representing the image, then initialize the timestamp
    # and occupied/unoccupied text
    image = frame.array
    # show the frame
    cv2.imshow("Frame", image)
    key = cv2.waitKey(1) & 0xFF
    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)
    # if the `q` key was pressed, break from the loop
    if key == ord("q"):
        break
#    elif key == ord("r"):
#        camera.resolution = saveResolution
#        rawCapture = PiRGBArray(camera, size=saveResolution)
#        time.sleep(0.1)
