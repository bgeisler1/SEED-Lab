# This code takes a video continuously, 
# looks for yellow, isolates yellow with a mask, 
# displays yellow shape by itself continuously 
# and finds center of shape and displays center as well as the location 
# of the center of the yellow and angle needed to be facing the shape. 
# Also finds quadrant of hexagon and shares information with an Arduino. 

 # defining and initializing the export variable 

exportVariable = 0 

#importing libraries 
from picamera import PiCamera 
from time import sleep 
import numpy as np 
import cv2 as cv 
import smbus 
import board 
import time 
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd 

# for RPI version 1, use “bus = smbus.SMBus(0)” 
bus = smbus.SMBus(1)

# This is the address we setup in the Arduino Program 
#address = 0x04

 #LCD display 
# Modify this if you have a different sized Character LCD 
lcd_columns = 16 
lcd_rows = 2 

# Initialise I2C bus. 
i2c = board.I2C()  # uses board.SCL and board.SDA 

 # Initialise the LCD class 
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows) 

#this function defines how to write number 
#def writeNumber(value): 
#    bus.write_byte(address, value) 
#    return -1 
    
#this function defines how to read number  
#def readNumber(address): 
#    number = bus.read_byte(address) 
#    return number 

# video definition 
v = cv.VideoCapture(0) 

# scaling the video to be smaller 
scale = 100 
fourcc = cv.VideoWriter_fourcc(*'XVID') 
rv = cv.VideoWriter('my.avi', fourcc, 20.0, (int((scale/100)*640), int((scale/100)*480))) 

# debugging camera 
if not v.isOpened(): 
    print('Camera is not opening') 
else: 
    print('Camera has been opened') 
  
# continuous while loop for doing all the operations while video is running 
while(v.isOpened()): 

    ret, frame = v.read() 

    # scaling the read in video 
    if ret == True: 
        w = int(frame.shape[1] * scale/100) 
        h = int(frame.shape[0] * scale/100) 
        d = (w, h) 
        frame = cv.resize(frame, d, interpolation = cv.INTER_AREA) 

        # converting to hsv and greyscale for better reults 
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV) 
        gr = cv.cvtColor(frame, cv.COLOR_BGR2GRAY) 
 
        # defining yellow and mask as well as resulting isolated shape plus testing some other ranges
        #yellow
        #lower = np.array([15, 100, 100], dtype = "uint8") 
        #upper = np.array([30, 255, 255], dtype = "uint8") 
#blue 1
        #lower = np.array([50, 50, 50], dtype = "uint8")
        #upper = np.array([130, 255, 255], dtype = "uint8")
#blue 2        
        lower = np.array([95, 100, 20], dtype = "uint8")
        upper = np.array([130, 255, 255], dtype = "uint8")

        #lower = np.array([90, 50, 70], dtype = "uint8") 
        #upper = np.array([128, 255, 255], dtype = "uint8")  
        #lower = np.array([100, 150, 0], dtype = "uint8") 
        #upper = np.array([140, 255, 255], dtype = "uint8")         

        mask = cv.inRange(hsv, lower, upper) 
        output = cv.bitwise_and(frame, frame, mask = mask) 

        # doing some cleanup 
        kernal = np.ones((5, 5), np.uint8)
        img_gray = cv.cvtColor(output, cv.COLOR_BGR2GRAY)
        img_gray = cv.morphologyEx(img_gray, cv.MORPH_OPEN, kernal)
        img_gray = cv.morphologyEx(img_gray, cv.MORPH_CLOSE, kernal)
        thresh, binary_img = cv.threshold(img_gray, thresh = 0, maxval = 255, type = cv.THRESH_BINARY + cv.THRESH_OTSU)
        
        #ret, thresh = cv.threshold (img_gray, 0, 255, cv.THRESH_BINARY + cv.THRESH_OTSU)
        #opening1 = cv.morphologyEx(output, cv.MORPH_OPEN, kernal)
        #blur = cv.GaussianBlur(opening1, (5, 5), 0)
        #opening2 = cv.morphologyEx(opening1, cv.MORPH_OPEN, kernal)
        #opening3 = cv.morphologyEx(opening2, cv.MORPH_OPEN, kernal)
        #opening4 = cv.morphologyEx(opening3, cv.MORPH_OPEN, kernal) 
        # finding and drawing contours for fancy highlights and center of shape 
        #img_gray = cv.cvtColor(opening1, cv.COLOR_BGR2GRAY)
        
        num_white_labels, labels_white_img = cv.connectedComponents(binary_img)
        
        labels_display = cv.normalize(src = labels_white_img, dst = None, alpha = 0, beta = 255, norm_type = cv.NORM_MINMAX, dtype = cv.CV_8U)
        #mask = cv.inRange(hsv, lower, upper)
        mask = cv.inRange(binary_img, 100, 255)
        output = cv.bitwise_and(frame, frame, mask = mask)
        #mask = cv.inRange(output, lower, upper) 
        #output = cv.bitwise_and(frame, frame, mask = mask) 
        #ret, thresh = cv.threshold(img_gray, 50, 255,cv.THRESH_BINARY) #used mask originally
        #ret, thresh = cv.threshold (labels_display, 0, 255, cv.THRESH_BINARY + cv.THRESH_OTSU)
        imageo, contours, hierarchy = cv.findContours(binary_img, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE) 
        #image2 = cv.drawContours(output, contours, 0, (0,0,255),2) #used output originally
        
        image2 = cv.drawContours(output, contours, 0, (0,0,255),2)

        # finding center and angle of shape is there is actually a shape 
        # on the screen otherwise displays no markers found! 
        if 0 < len(contours): 
            My = cv.moments(contours[0]) 
            try: 
                cv.circle(output, (round(My['m10'] / My['m00']), round(My['m01'] / My['m00'])), 3, (0, 0, 255), -1) 
                #cv.imshow('frame', output)
                cv.imshow('frame', output) 
                rv.write(output) 
                print("Center of X: '{}'".format(round(My['m10'] / My['m00']))) 
                print("Center of Y: '{}'".format(round(My['m01'] / My['m00']))) 
# Making case statements to find which quadrant the center of the yellow hexagon is in and assigning it to the  
# export variable.                 

                if (round(My['m10'] / My['m00']) > 320) and (round(My['m01'] / My['m00']) < 240): 
                    quad = 1 
                    print("Quandrant: ", quad) 
                if (round(My['m10'] / My['m00']) < 320) and (round(My['m01'] / My['m00']) < 240): 
                    quad = 2 
                    print("Quandrant: ", quad) 
                if (round(My['m10'] / My['m00']) < 320) and (round(My['m01'] / My['m00']) > 240): 
                    quad = 3 
                    print("Quandrant: ", quad) 
                if (round(My['m10'] / My['m00']) > 320) and (round(My['m01'] / My['m00']) > 240): 
                    quad = 4 
                    print("Quandrant: ", quad)      
      
                hFov = 62.2 
                vFov = 48.8 
                mX = output.shape[1]/2 
                mY = output.shape[0]/2 
                hA = (((My['m10']/My['m00']) - (output.shape[1]/2))/ output.shape[1]) * hFov 
                vA = (((My['m01']/My['m00']) - (output.shape[0]/2))/ output.shape[0]) * vFov 
                print("Angle of Phi: '{}'deg".format(round(hA, 1))) 
                print("Angle of Theta: '{}'deg".format(round(vA, 1)))
                
                exportVariable = format(round(hA, 1))

            except: 
                cv.putText(frame, 'No Markers Found!', (5, 15), cv.FONT_HERSHEY_TRIPLEX, 1, (255, 255, 255, 255), 3) 
                cv.imshow('frame', frame) 
                rv.write(frame) 
        else: 
            cv.putText(frame, 'No Markers Found!', (5, 15), cv.FONT_HERSHEY_TRIPLEX, 1, (255, 255, 255, 255), 3) 
            cv.imshow('frame', frame) 
            rv.write(frame) 

        # if user hits escape the program stops showing video cool     
        if cv.waitKey(1) & 0xFF == 27: 
            break 
            
    # just a debugging mechanism     
    else: 
        print('The frame is not being recieved!, Toodles...') 
        break 
        
   # writeNumber(exportVariable) 

    # Set LCD color to red 
    while True: 
        lcd.message = "Angle: %s"%(exportVariable) 
        break 

# cleaning up the program     

v.release() 
rv.release() 
cv.destroyAllWindows() 
