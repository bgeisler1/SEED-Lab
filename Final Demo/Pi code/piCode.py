# This code takes a video continuously, 
# looks for blue, isolates blue with a mask, 
# displays blue shape by itself continuously 
# and finds center of shape and displays center as well as the location 
# of the center of the bliue and angle needed to be facing the shape. 
# Also finds quadrant of blue shape and shares information with an Arduino. 



#importing libraries 
from picamera import PiCamera 
from time import sleep 
import numpy as np 
import cv2 as cv 
import smbus 
import board 
import time 
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

#defining global variables

crossFound = 0
rightTurn = 0

jiuzou = 10
ben = 12
robert = 100
benNegative = 0
jiuzouNegative = 0
exportVariable = 0
markersFound = 0
dist = 0
totDist = 0
alpha = 12
camh = .17
#caml = camh * tan(alpha)
limit = 0
hA = 0
vA = 0


framerate = 10
prev = 0
# for RPI version 1, use “bus = smbus.SMBus(0)” 
bus = smbus.SMBus(1)

# This is the address we setup in the Arduino Program 
address = 0x04

 #LCD display 
# Modify this if you have a different sized Character LCD 
lcd_columns = 16 
lcd_rows = 2 

# Initialise I2C bus. 
i2c = board.I2C()  # uses board.SCL and board.SDA 

 # Initialise the LCD class 
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows) 
array = [robert, ben, benNegative, jiuzou, jiuzouNegative]
#this function defines how to write number 
#def writeNumber(value,array): 
    #bus.write_i2c_block_data(address, value, array) 
    #return -1
def writeNumber(value): 
    bus.write_byte(address, value) 
    return -1 
    
#this function defines how to read number  
def readNumber(address): 
    number = bus.read_byte(address) 
    return number 

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

    time_elapsed = time.time() - prev
    ret, frame = v.read() 
    
    if time_elapsed > 1./framerate:
        prev = time.time()
        
        # scaling the read in video 
        if ret == True: 
            w = int(frame.shape[1] * scale/100) 
            h = int(frame.shape[0] * scale/100) 
            d = (w, h) 
            frame = cv.resize(frame, d, interpolation = cv.INTER_AREA) 

            # converting to hsv and greyscale for better reults 
            hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV) 
            gr = cv.cvtColor(frame, cv.COLOR_BGR2GRAY) 
     
            # defining blue and mask as well as resulting isolated shape plus testing some other ranges
      
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
            
            
            num_white_labels, labels_white_img = cv.connectedComponents(binary_img)
            
            labels_display = cv.normalize(src = labels_white_img, dst = None, alpha = 0, beta = 255, norm_type = cv.NORM_MINMAX, dtype = cv.CV_8U)

            mask = cv.inRange(binary_img, 100, 255)
            output = cv.bitwise_and(frame, frame, mask = mask)

            imageo, contours, hierarchy = cv.findContours(binary_img, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE) 

            image2 = cv.drawContours(output, contours, 0, (0,0,255),2)

            # finding center and angle of shape is there is actually a shape 
            # on the screen otherwise displays no markers found! 
            if 0 < len(contours): 
                My = cv.moments(contours[0]) 
                try: 
                    cv.circle(output, (round(My['m10'] / My['m00']), round(My['m01'] / My['m00'])), 3, (0, 0, 255), -1)

                    cv.imshow('frame', output) 
                    rv.write(output) 
                    #print("Center of X: '{}'".format(round(My['m10'] / My['m00']))) 
                    #print("Center of Y: '{}'".format(round(My['m01'] / My['m00'])))
                    markersFound = 1
    # Making case statements to find which quadrant the center of the blue is in and assigning it to the  
    # export variable.                 

                    if (round(My['m10'] / My['m00']) > 320) and (round(My['m01'] / My['m00']) < 240): 
                        quad = 1 
                        #print("Quandrant: ", quad) 
                    if (round(My['m10'] / My['m00']) < 320) and (round(My['m01'] / My['m00']) < 240): 
                        quad = 2 
                        #print("Quandrant: ", quad) 
                    if (round(My['m10'] / My['m00']) < 320) and (round(My['m01'] / My['m00']) > 240): 
                        quad = 3 
                        #print("Quandrant: ", quad) 
                    if (round(My['m10'] / My['m00']) > 320) and (round(My['m01'] / My['m00']) > 240): 
                        quad = 4 
                        #print("Quandrant: ", quad)
                        
                        # Finding some other values that were not really used
                        
                    if (round(My['m10'] / My['m00']) < 100) and (round(My['m10'] / My['m00']) > 380):
                        crossFound = 1
                        print("Cross Found!")
                    elif (round(My['m10'] / My['m00']) > 380) and (round(My['m01'] / My['m00']) > 420):
                        rightTurn = 1
                        print("Right Turn Incoming!")
                    if (round(My['m01'] / My['m00']) > 440):
                        quad = 5
                        limit = 1
                        print("At Vision Limit!")
                    
                    
                    
          
                    hFov = 62.2 
                    vFov = 48.8 
                    mX = output.shape[1]/2 
                    mY = output.shape[0]/2 
                    hA = (((My['m10']/My['m00']) - (output.shape[1]/2))/ output.shape[1]) * hFov 
                    vA = (((My['m01']/My['m00']) - (output.shape[0]/2))/ output.shape[0]) * vFov 
                    #print("Angle of Phi: '{}'deg".format(round(hA, 1))) 
                    #print("Angle of Theta: '{}'deg".format(round(vA, 1)))
                    
                    exportVariable = format(round(hA, 1))
                    
                    #dist = camh * tan((pi/2) - alpha - vA)
                    #totDist = dist + caml
                    totDist = 1

                except: 
                    cv.putText(frame, 'No Markers Found!', (5, 15), cv.FONT_HERSHEY_TRIPLEX, 1, (255, 255, 255, 255), 3) 
                    cv.imshow('frame', frame) 
                    rv.write(frame)
                    markersFound = 0
            else: 
                cv.putText(frame, 'No Markers Found!', (5, 15), cv.FONT_HERSHEY_TRIPLEX, 1, (255, 255, 255, 255), 3) 
                cv.imshow('frame', frame) 
                rv.write(frame)
                markersFound = 0

            # if user hits escape the program stops showing video cool     
            if cv.waitKey(1) & 0xFF == 27: 
                break 
                
        # just a debugging mechanism     
        else: 
            print('The frame is not being recieved!, Toodles...') 
            break 
        
        if (markersFound == 0):
            vA = 0
            hA = 0
            
        robert = markersFound
        ben = int(round(hA, 0))
        jiuzou = int(round(vA, 0))
        #print(robert)
        #print(ben)
        #print(jiuzou)
        
        benNegative = 0
        jiuzouNegative = 0
        
        if (ben < 0):

            benNegative = 1

        
        if (jiuzou < 0):

            jiuzouNegative = 1

     
        #writeNumber(totDist)
        #writeNumber(markersFound)
        #writeNumber(exportVariable)
        imReady = readNumber(address)
        if (imReady == 1):
            #writeNumber(markersFound, array) #This is in charge of writing to the Arduino
            #if (benNegative == 1)
            lcd.message = "Phi: %d"%(hA)
            lcd.message = "\n Theta: %d" %(vA)
            writeNumber(robert)
            writeNumber(ben)
            writeNumber(benNegative)
            writeNumber(jiuzou)
            writeNumber(jiuzouNegative)
            #writeNumber(limit)
        #writeNumber(ben)
        #writeNumber(benNegative)
        #writeNumber(jiuzou)
        #writeNumber(jiuzouNegative)
        # Set LCD color to red 
        #while True: 
            #break 

    # cleaning up the program     

v.release() 
rv.release() 
cv.destroyAllWindows() 



#totDist is the distance between wheels and center of the tape
#markersFound is whether or not we see tape
#exportVariable is the horizontal angle to the center of the tape

#limit is a boolean that says whether or not the marker is at the vision limit and calculation for final push should be made


#crossFound is a variable that sets to 1 when the wings of the cross are detected on either side of the frame of view of the camera
#rightTurn is much like cross found but activates if crossfound is not true and also only activates if the tape is located at the bottom right of the screen
