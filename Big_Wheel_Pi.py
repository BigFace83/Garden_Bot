import time
import serial
import os
import numpy as np
import cv2
import RPi.GPIO as GPIO
import math


print "Starting..."

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(17,GPIO.OUT) #Red LED
GPIO.setup(22,GPIO.OUT) #Blue LED
GPIO.setup(27,GPIO.OUT) #Green LED

GPIO.setup(21, GPIO.IN, pull_up_down=GPIO.PUD_UP) #Power off button

recording = False
Speed = 0
Angle = 0
Servo1 = 90
Servo2 = 90

print ("Opening Arduino connection")
serArd = serial.Serial('/dev/ttyUSB0', 115200, timeout = 10)
time.sleep(2) # wait here a bit, let arduino boot up fully before sending data
print ("Port open:  " + serArd .portstr)       # check which port was really used
time.sleep(2)
serArd.flushInput()

print ("Opening HC05 connection")
serHC05 = serial.Serial('/dev/ttyAMA0', 9600, timeout = 10)
time.sleep(2) # wait here a bit, let arduino boot up fully before sending data
print ("Port open:  " + serHC05 .portstr)       # check which port was really used
serHC05.flushInput()

print "Starting OpenCV"
capture = cv2.VideoCapture(0)

#################################################################################
#For Visual encoders
orb = cv2.ORB_create(2000)
bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True) #Brute force matcher
calib = 1000.0/400
#################################################################################


imagewidth = 320
imageheight = 240
capture.set(3,imagewidth) #1024 640 1280 800 384
capture.set(4,imageheight) #600 480 960 600 288

# Define the codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'XVID')

cv2.waitKey(50)

#################################################################################
#Visual encoder functions
def FindORB(img):
    kp = orb.detect(img,None) # find the keypoints with ORB
    kp, des = orb.compute(img, kp) # compute the descriptors with ORB
    # draw only keypoints location,not size and orientation
    #img2 = cv2.drawKeypoints(img,kp,img,color=(0,255,0), flags=0)
    return img, kp, des

def Match(kp0, kp1, des0, des1):
    matches = bf.match(des0, des1)
    good = []
    for i, m in enumerate(matches):
        if i < len(matches) - 1 and m.distance < 0.5 * matches[i+1].distance:
            good.append(m)
    return good

def Averageflow(matches, kp0, kp1):

    avdist = 0
    avangle = 0
    disttot = 0
    angtot = 0

    for x in matches:
        point0 = kp0[x.queryIdx].pt
        point1 = kp1[x.trainIdx].pt

        y = point1[0] - point0[0]
        x = point1[1] - point0[1]
        dist = math.sqrt((y*y)+(x*x))
        disttot += dist

        angle = math.degrees(-math.atan2(y, x))
        angtot += angle

    avdist = disttot/len(matches)
    avangle = angtot/len(matches)

    y = int(math.sin(math.radians(-angle))*dist)
    x = int(math.cos(math.radians(-angle))*dist)

    return avdist, x, y, avangle

def DistfromVision(img1, img2):
    frame1, kp1, des1 = FindORB(img1)
    frame2, kp2, des2 = FindORB(img2)
    
    print 'Keypoint lengths', len(kp1), len(kp2)

    if (len(kp1) > 1) and (len(kp2) > 1):
        matches = Match(kp1, kp2, des1, des2)
        print 'Match length', len(matches)
        if len(matches) > 1:
            avdist, x, y, avang = Averageflow(matches, kp1, kp2)
            return avdist*calib
                
        else:
            return None
    else:
        return None
        

   
#################################################################################
def CaptureFrame():
    ret,img = capture.read()
    ret,img = capture.read()
    ret,img = capture.read()
    ret,img = capture.read()
    ret,img = capture.read() #Read a bunch of frames to make sure we have the latest image
    cv2.waitKey(1)
    return img

def LEDBlue():
    GPIO.output(17,GPIO.HIGH) 
    GPIO.output(22,GPIO.HIGH) 
    GPIO.output(27,GPIO.LOW)
    
def LEDRed():
    GPIO.output(17,GPIO.HIGH)
    GPIO.output(22,GPIO.LOW)
    GPIO.output(27,GPIO.HIGH)

def LEDGreen():
    GPIO.output(17,GPIO.LOW)
    GPIO.output(22,GPIO.HIGH)
    GPIO.output(27,GPIO.HIGH)

def LEDOff():
    GPIO.output(17,GPIO.HIGH)
    GPIO.output(22,GPIO.HIGH)
    GPIO.output(27,GPIO.HIGH)

def CreateFiles():
    print "Creating files for writing"
    timestr = time.strftime("%Y%m%d-%H%M%S")
    os.chdir("/home/pi/RC_Robot")
    try:
        os.mkdir(timestr, 0o777)
    except OSError:
        print "Creation of directory failed"

    os.chdir(timestr)
    out = cv2.VideoWriter(timestr +'.avi',fourcc, 10.0, (imagewidth,imageheight))
    f = open("Data.txt", 'w')
    return out, f

def Shutdown(event):
    print("Shutting Down")
    LEDOff()
    time.sleep(0.5)
    LEDBlue()
    time.sleep(0.5)
    LEDOff()
    time.sleep(0.5)
    LEDBlue()
    time.sleep(0.5)
    LEDOff()
    time.sleep(0.5)
    LEDBlue()
    time.sleep(0.5)
    LEDOff()
    time.sleep(0.5)
    LEDRed()
    os.system("sudo shutdown -h now")

def readserial():
    #while True:
    while serHC05.inWaiting()>0: #while data is available
        line = serHC05.readline() # keep reading until...
        if serHC05.inWaiting()==0: #...all data is read. Previous data is discarded to keep robot responsive
            return line.rstrip('\r\n') #Return most recent data
    

def sendcommand(serialdata):
    serArd.write(serialdata + '\n') #Write data string, newline terminated

def readserialArd():
    while serArd.inWaiting()==0: #Wait here until data
        pass
    while serArd.inWaiting()>0: #while data is available
        line = serArd.readline() # keep reading until...
        if serArd.inWaiting()==0: #...all data is read. Previous data is discarded to keep robot responsive
            return line.rstrip('\r\n') #Return most recent data
    
'''
def readserialArd():
    if serArd.inWaiting()>0: #If data is available
        line = serArd.readline()
        return line.rstrip('\r\n') #Return packet
    else: #If no data to read, return None
        return None
'''

def flushserialArd():
    while serArd.inWaiting()>0: #If data is available
        line = serArd.readline() #Read all data in buffer, discard old data

def closeserial():
    ser.close()

def LEDredflash(): #Flash LED red 
    for x in range (0, 1, 1):
        LEDRed()
        time.sleep(0.1)
        LEDOff()
        time.sleep(0.1)

def Scan(out, f, imagecounter):
    arddata = readserialArd() #get most up to date values from Arduino
    if arddata is not None:
        ardarray = [int(x) for x in data.split(',')]
        yaw = ardarray[2]
        Servo1 = 0
        Servo2 = 90
        SetHeadPos(Servo1, Servo2)
        CaptureandSave(f, imagecounter, yaw, Servo1, Servo2)
        imagecounter+=1
        Servo1 = 45
        SetHeadPos(Servo1, Servo2)
        CaptureandSave(f, imagecounter, yaw, Servo1, Servo2)

        imagecounter+=1
        Servo1 = 90
        SetHeadPos(Servo1, Servo2)
        CaptureandSave(f, imagecounter, yaw, Servo1, Servo2)

        imagecounter+=1
        Servo1 = 135
        SetHeadPos(Servo1, Servo2)
        CaptureandSave(f, imagecounter, yaw, Servo1, Servo2)

        imagecounter+=1
        Servo1 = 180
        SetHeadPos(Servo1, Servo2)
        CaptureandSave(f, imagecounter, yaw, Servo1, Servo2)

        imagecounter+=1
        Servo1 = 90
        SetHeadPos(Servo1, Servo2)
 
        return imagecounter
    

def CaptureandSave(f, name, yaw, Servo1, Servo2):
    img = CaptureFrame()
    filename = "%d.jpg"%name
    cv2.imwrite(filename, img)     # save frame as JPEG file
    f.write(str(1) + "," + str(yaw) + "," + str(Servo1) + "," + str(Servo2) + "," + str(name) + '\n')

def Datatofile(f, yaw, Servo1, Servo2):
    f.write(str(0) + "," + str(yaw) + "," + str(Servo1) + "," + str(Servo2) + '\n')

def SetHeadPos(Servo1, Servo2):
    sendcommand(str(3) + "," + str(Servo1))
    sendcommand(str(4) + "," + str(Servo2))
    time.sleep(1)

def Turn(angle, f):
    arddata = readserialArd() #get most up to date values from Arduino
    if arddata is not None:
        ardarray = [int(x) for x in arddata.split(',')]
        yaw = ardarray[2]
        
        newyaw = yaw + angle #Turn robot 180 degrees
        if (newyaw > 179):
            newyaw = newyaw - 359
        elif (newyaw < -179):
            newyaw = newyaw + 359
        print yaw, newyaw
        sendcommand(str(2) + "," + str(newyaw))

        while True:
            arddata = readserialArd() #get most up to date values from Arduino
            if arddata is not None:
                ardarray = [int(x) for x in arddata.split(',')]
                yaw = ardarray[2]
               
                if (yaw == newyaw):
                    Datatofile(f, yaw, Servo1, Servo2)
                    sendcommand(str(0) + "," + str(0))
                    break

   
def Drive(speed, dist):
    totaldist = 0
    imgold = CaptureFrame()
    SetHeadPos(90, 60)
    sendcommand(str(1) + "," + str(speed))
    lastdist = 0
    while True:
        imgnew = CaptureFrame()
        estdist = DistfromVision(imgold, imgnew)
        if estdist is not None:
            totaldist += estdist
            lastdist = estdist   
        else:
            totaldist += lastdist
        print str(totaldist) + "mm"
        if totaldist >= dist:
            break
        imgold = imgnew


    sendcommand(str(0) + "," + str(0))
    


GPIO.add_event_detect(21, GPIO.FALLING, callback=Shutdown, bouncetime=2000)


LEDBlue()
arddata = readserialArd() #Wait here on start-up until data is received from Arduino
time.sleep(1) #Wait a sec...
flushserialArd() #Clear input buffer ready for good data

LEDGreen()



while True:

    data = readserial()
    if data is not None:
        dataarray = [int(x) for x in data.split(',')]
        if dataarray[5] == 0: #Right button clicked
            LEDRed()
            out, f = CreateFiles()
            arddata = readserialArd() #get most up to date values from Arduino
            print arddata
            if arddata is not None:
                ardarray = [int(x) for x in data.split(',')]
                yaw = ardarray[2]
                Datatofile(f, yaw, Servo1, Servo2)
            #imcount = Scan(out, f, 0)
            #Turn(180, f)
            #imcount = Scan(out, f, imcount)
            #Turn(180, f)
            #imcount = Scan(out, f, imcount)
            Drive(220, 1000)
            #imcount = Scan(out, f, imcount)
            LEDGreen()
            f.close()
            print 'Sequence Complete'
            























 





