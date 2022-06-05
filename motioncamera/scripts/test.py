# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import json
from numpy import array, zeros, ones, isclose
from numpy import max as MAX
# initialize the camera and grab a reference to the raw camera capture
# Resolution used when checking for motion (low resolution for faster searching)
def loadConfig(fpath):
    return json.load(open(fpath, 'r'))
def GetToggledResolution(camera, config):
    width = str(camera.resolution).split('x')[0]
    height = str(camera.resolution).split('x')[1]

    if int(width) == config["searchResolution"]["width"] and  int(height) == config["searchResolution"]["height"]:
        return config["saveResolution"]["width"], config["saveResolution"]["height"]
        
    return config["searchResolution"]["width"], config["searchResolution"]["height"]

class MotionDetector:
    """
    Checks difference between camera frames frames to detect motion
    """
    _frameHeight = None
    _frameWidth = None
    _frame = None
    _lastMotionDetectedTime = None

    def __init__(self, frameWidth, frameHeight):
        self._frameHeight = frameHeight
        self._frameWidth = frameWidth
        self._frame = zeros((frameWidth, frameHeight))
    
    def CaluculateFrameDifference(self, newFrame):
        """
        Get the difference beween frames
        """
        
        pass


    def DetectMotion(self, newFrame):
        diff = abs(self._frame - newFrame)
        print(diff)
        self._frame = newFrame
        kernel = ones((5, 5))
        diff = cv2.dilate(diff, kernel, 1)
        return diff 
        
# Resolution used to check for motion in the frame

def main():
    config = loadConfig('../config/camera.conf.json')
    searchResolution = (config["searchResolution"]["width"], config["searchResolution"]["height"])
    saveResolution = (config["saveResolution"]["width"], config["saveResolution"]["height"])
    camera = PiCamera()
    camera.resolution = searchResolution
    camera.framerate = 32
    rawCapture = PiRGBArray(camera, size=searchResolution)
    # allow the camera to warmup
    time.sleep(0.1)
    motionDetector = MotionDetector(searchResolution[0], searchResolution[1])
    # capture frames from the camera
    lastDetectionTime = time.time()
    final_frame = zeros((searchResolution[0], searchResolution[1])) 
    while True:
        for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
            # grab the raw NumPy array representing the image, then initialize the timestamp
            # and occupied/unoccupied text
            image = frame.array
            # show the frame
            gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            if time.time() - lastDetectionTime > config.get("delayStepTimeTimeout", 0.1):
                lastDetectionTime = time.time()
                final_frame = motionDetector.DetectMotion(gray_image)

            cv2.imshow("Frame_diff", final_frame)
            cv2.imshow("Frame_real", gray_image)
            
            key = cv2.waitKey(1) & 0xFF

            # clear the stream in preparation for the next frame
            rawCapture.truncate(0)
            # if the `q` key was pressed, break from the loop
            if key == ord("q"):
                return
            elif key == ord("r"):
                newResolution = GetToggledResolution(camera, config)
                camera.resolution = newResolution
                rawCapture = PiRGBArray(camera, size=newResolution)
                time.sleep(0.1)
                break

if __name__ == "__main__":
    main()
