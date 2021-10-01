from picamera import PiCamera
from time import sleep
import numpy as np
import cv2 as cv
#from matplotlib import pyplot as plt

v = cv.VideoCapture(0)

scale = 60
fourcc = cv.VideoWriter_fourcc(*'XVID')
rv = cv.VideoWriter('my.avi', fourcc, 20.0, (int((scale/100)*640), int((scale/100)*480)))

if not v.isOpened():
    print('Camera is not opening')
else:
    print('Camera has been opened')

while(v.isOpened()):
    ret, frame = v.read()
    
    if ret == True:
        w = int(frame.shape[1] * scale/100)
        h = int(frame.shape[0] * scale/100)
        d = (w, h)
        frame = cv.resize(frame, d, interpolation = cv.INTER_AREA)
        
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        gr = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        
        lower = np.array([15, 100, 100], dtype = "uint8")
        upper = np.array([30, 255, 255], dtype = "uint8")
        
        mask = cv.inRange(hsv, lower, upper)
        output = cv.bitwise_and(frame, frame, mask = mask)
        
        kernal = np.ones((5, 5), np.uint8)
        opening = cv.morphologyEx(output, cv.MORPH_OPEN, kernal)
        
        #cv.imwrite("msk.jpg", mask)
        img_gray = cv.cvtColor(opening, cv.COLOR_BGR2GRAY)
        ret, thresh = cv.threshold(img_gray, 50, 255,cv.THRESH_BINARY) #used mask originally
        imageo, contours, hierarchy = cv.findContours(thresh, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        image2 = cv.drawContours(output, contours, 0, (0,0,255),2) #used output originally
        
        if 0 < len(contours):
            My = cv.moments(contours[0])
            try:
                cv.circle(output, (round(My['m10'] / My['m00']), round(My['m01'] / My['m00'])), 3, (0, 0, 255), -1)
                cv.imshow('frame', output)
                rv.write(output)
                print("Center of X: '{}'".format(round(My['m10'] / My['m00'])))
                print("Center of Y: '{}'".format(round(My['m01'] / My['m00'])))
                
                hFov = 62.2
                vFov = 48.8
                mX = output.shape[1]/2
                mY = output.shape[0]/2
                hA = (((My['m10']/My['m00']) - (output.shape[1]/2))/ output.shape[1]) * hFov
                vA = (((My['m01']/My['m00']) - (output.shape[0]/2))/ output.shape[0]) * vFov
                
                print("Angle of Phi: '{}'deg".format(round(hA, 1)))
                print("Angle of Theta: '{}'deg".format(round(vA, 1)))
                
            except:
                cv.putText(frame, 'No Markers Found!', (5, 15), cv.FONT_HERSHEY_TRIPLEX, 1, (255, 255, 255, 255), 3)
                cv.imshow('frame', frame)
                rv.write(frame)
        else:
            cv.putText(frame, 'No Markers Found!', (5, 15), cv.FONT_HERSHEY_TRIPLEX, 1, (255, 255, 255, 255), 3)
            cv.imshow('frame', frame)
            rv.write(frame)
            
        if cv.waitKey(1) & 0xFF == 27:
            break
    else:
        print('The frame is not being recieved!, Toodles...')
        break
    
v.release()
rv.release()
cv.destroyAllWindows()
                
        
        
        
                    