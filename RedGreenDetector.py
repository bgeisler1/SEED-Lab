from picamera.array import PiRGBArray
import picamera

import numpy as np
import time
import cv2

def take_picture(Output):
    rawCapture.truncate(0)
    camera.capture(rawCapture, format="bgr")
    image = rawCapture.array
    return process_image(image, Output)

def process_image(image, OutPut):
    print("Processing image...")
    image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    if OutPut == False: #looking for green
        mask = cv2.inRange(image, lower_green, upper_green)

    elif OutPut == True: #looking for red
        uppermask = cv2.inRange(image, lower_redhigh, upper_redhigh)
        lowermask = cv2.inRange(image, lower_redlow, upper_redlow)
        mask = uppermask + lowermask

        #Use closing morphological transformation to fill in holes
        closedImg = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernal)

        #Use opening morphological transformation to eliminate small noise
        openedImg = cv2.morphologyEx(closedImg, cv2.MORPH_OPEN, kernal)

        return blob_detection(openedImg, OutPut)

def blob_detection(image, OutPut):
    keyPoints = detector.detect(image)
    print("Found this keyPoint:")
    print(keyPoints)
    print("at time %d" % time.clock())
    if keyPoints == []:
        OutPut = OutPut
        print(OutPut)
    else:
        OutPut = not OutPut
        print(OutPut)
    return take_picture(OutPut)

#Init State 1
#Calibration step: In order to take consistent images we must ensure that our
#exposure time, white balance, and gains are all fixed.
camera = picamera.PiCamera()
camera.resolution = (1280, 720)
camera.framerate = 30
camera.sensor_mode = 5

#LEDs produce a lot of light. Fast shutter speed and low iso.
camera.shutter_speed = 20000
camera.iso = 200
camera.awb_mode = "off"
camera.awb_gains = (343/256, 101/64)

#Now that the camera is initialized lets make a reference to the raw cam capture
rawCapture = PiRGBArray(camera)

#These values should be made global to prevent repeated init
lower_green = np.array([55,230,130])
upper_green = np.array([65,255,255])

lower_redhigh = np.array([160,170,200])
upper_redhigh = np.array([179,255,255])
lower_redlow = np.array([0,170,200])
upper_redlow = np.array([10,255,255])

#kernal is used in the noise cancelling process
kernal = cv2.getStructuringElement(cv2.MORPH_CROSS, (4,4))

params = cv2.SimpleBlobDetector_Params()

params.filterByColor = True
params.blobColor = 255

params.thresholdStep = 10
params.minThreshold = 50
params.maxThreshold = 220
params.minRepeatability = 1
minDistBetweenBlobs = 10
params.filterByArea = 1
params.minArea = 30
params.maxArea = 3000
#Filter by Circularity (looking for near circles)
params.filterByCircularity = 1
params.minCircularity = .5
#Filter by Convexity (off)
params.filterByConvexity = 0
params.filterByInertia = 0

detector = cv2.SimpleBlobDetector_create(params)

OutPut = False
take_picture(OutPut)

















    
