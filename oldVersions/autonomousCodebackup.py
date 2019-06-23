#####REMINDER: GPIO MODE BOARD
import os
from gps import *
from time import *
import time
import math
from math import sin, cos, sqrt, atan2, radians
 
import smbus
import threading
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BOARD)
gpsd = None #seting the global variable

#bus = smbus.SMBus(1)
#address = 0x1e
##############Serial####################
import serial
port = serial.Serial("/dev/ttyAMA0", baudrate=57600, timeout=0.5)

############SERVO###################
######################################
GPIO.setup(7, GPIO.OUT)
p=GPIO.PWM(7, 50)
p.start(7.5)

###############Variables################
#######################################
count=1

pointing=0
shouldPoint=0

whatToDo=0
distance=0
goingToLocation=0

latitude=0
longitude=0
utctime=0
speedGPS=0

###############MOTORPINS################
#######################################
Motor1A=13
Motor1B=15
Motor2A=16
Motor2B=18

GPIO.setup(Motor1A, GPIO.OUT)
GPIO.setup(Motor1B, GPIO.OUT)
GPIO.setup(Motor2A, GPIO.OUT)
GPIO.setup(Motor2B, GPIO.OUT)

##THRUSTER
#byte servoPin = 7; // thruster
#Servo servo;


#https://www.darrinward.com/lat-long/?id=5a76032f24a6e2.23164443
#MSU Space
#waypoint = [38.190809, -83.430195,
#            38.190839, -83.430343,
#            38.190723, -83.430474,
#            38.190559, -83.430378,
#            38.190593, -83.430087]

#Home
waypoint = [38.197861, -83.407252,
38.197762, -83.407334,
38.197824, -83.407465,
38.197912, -83.407622,
38.198085, -83.407491]

##WAYPOINTS
finalX = waypoint[0]
finalY = waypoint[1]
finalX2 = waypoint[2]
finalY2 = waypoint[3]
finalX3 = waypoint[4]
finalY3 = waypoint[5]
finalX4 = waypoint[6]
finalY4 = waypoint[7]
finalX5 = waypoint[8]
finalY5 = waypoint[9]

#################################### METHODS BEGIN HERE######################
#############################################################################
def multiplePoint(a1,b1,a2,b2,a3,b3,a4,b4,a5,b5):
    if count == 1:
        fromPointToPoint(latitude, longitude, a1, b1)
        global goingToLocation
	goingToLocation='1st'
	#print '->1st'
    elif count == 2:
        fromPointToPoint(latitude, longitude, a2, b2)
        global goingToLocation
        goingToLocation='2nd'
	#print '->2nd'
    elif count == 3:
        fromPointToPoint(latitude, longitude, a3, b3)
        global goingToLocation
        goingToLocation='3rd'
	#print '->3rd'
    elif count == 4:
        fromPointToPoint(latitude, longitude, a4, b4)
        #print '->4th'
        global goingToLocation
        goingToLocation='4th'
    elif count == 5:
        fromPointToPoint(latitude, longitude, a5, b5)
        #print '->5th'
        global goingToLocation
        goingToLocation='5th'
    else:
        global goingToLocation
        goingToLocation='REACHED'
        stopTheEngine()


def fromPointToPoint(x, y, a, b):
    if (x < a + 0.0000099 and x > a - 0.0000099 and y < b + 0.0000099 and y > b - 0.0000099): #is the car at the fin.dest?
        count+=1
    else:
        turnDegrees(x, y, a, b)#calculates/thinks where the car should point
        #print("ANGLE DIFF:")
        if (shouldPoint > pointing):
            print "ANGLE DIFF:",(shouldPoint - pointing)
            print("     UTURN:")
            if (80<shouldPoint - pointing and shouldPoint - pointing < 100):
                print("YES")
            else:
                print("N/A")
        elif (shouldPoint < pointing):
            print "ANGLE DIFF:",(pointing - shouldPoint)
            print("     UTURN:")
            if (80<pointing - shouldPoint and pointing - shouldPoint < 100):
                print("YES")
            else:
                print("N/A")
        print("   \n")
###############DISTANCE CALCULATE##########
        calcDist(x, y, a, b)
        #print("**\n")
        if (pointing > shouldPoint - 20 and pointing < shouldPoint + 20 ): ##is it already pointing at that point?
            forward()
            backToZero()
            global whatToDo
            whatToDo=("Matching/Just go fwd \n")
            #print("Matching/Just go fwd \n")
        else:
            turnToWhichSide(pointing, shouldPoint) ##look the fin.dest way
            global whatToDo
            whatToDo=("Need to turn \n")
            #print("Need to turn \n")

def turnDegrees(x, y, a, b):
    num1 = (math.fabs(x))
    num2 = (math.fabs(y))
    num3 = (math.fabs(a))
    num4 = (math.fabs(b))
#    print("\n")
    if (x < a and y < b): # top right
        #print("Option4\n")
        global shouldPoint
	shouldPoint = 90.0 - ( math.atan2 (math.fabs(num4 - num2), math.fabs(num3 - num1)) * 180.0 / 3.14159265 )
    elif (x < a and b < y): # bottom right
        #print("Option1\n")
	global shouldPoint
        shouldPoint = 90.0 + ( math.atan2 (math.fabs(num4 - num2), math.fabs(num3 - num1)) * 180.0 / 3.14159265 )
    elif (a < x and b < y):  # bottom left
        #print("Option2\n")
	global shouldPoint
        shouldPoint = 270.0- ( math.atan2 (math.fabs(num4 - num2), math.fabs(num3 - num1))  * 180.0 / 3.14159265 )
    elif (a < x and y < b):  #top left
        #print("Option3\n")
	global shouldPoint
        shouldPoint = 270.0 + ( math.atan2 (math.fabs(num4 - num2), math.fabs(num3 - num1))  * 180.0 / 3.14159265 )



def turnToWhichSide( x, a ): #calculates turning to which side is the shortest
    if(x<180):
        if (x<a and a<x+180):
            right()
            forward()
            port.write("Right")
            print "Right"
        else: 
            left()
            forward()
            port.write("Left")
            print "Left"
    else:
        if (x-180<a and a<x):
            left()
            forward()
            port.write("Left")
            print "Left"
        else:
            right()
            forward()
            port.write("Right")
            print "Right"

def calcDist( x,  y,  a,  b):
  num1 =  (math.fabs(x))
  num2 =  (math.fabs(y))
  num3 =  (math.fabs(a))
  num4 =  (math.fabs(b))

  #Each degree of  latitudeX is approximately 69 miles=111.045 kilometers=4,371,840 Inches=4,371,850.393701 Inches=364320ft
  #Each degree of longitudeY is approximately 69.172 miles=111.321543 kilometers=4382737.913386inches=365228.16ft
  xDiff =  (math.fabs(num1 - num3))#long
  yDiff =  (math.fabs(num2 - num4))#lat
  #now to inches
  xDiff = xDiff * 364320
  yDiff = yDiff * 365228.16
  dist = math.sqrt(xDiff * xDiff + yDiff * yDiff)

  a = sin(dlat / 2)**2 + cos(lat1) * cos(lat2) * sin(dlon / 2)**2
  c = 2 * atan2(sqrt(a), sqrt(1 - a))

  distance = R * c*3280.84

  global distance
  distance=dist

#######################GPS##########################################
class GpsPoller(threading.Thread):
  def __init__(self):
    threading.Thread.__init__(self)
    global gpsd #bring it in scope
    gpsd = gps(mode=WATCH_ENABLE) #starting the stream of info
    self.current_value = None
    self.running = True #setting the thread running to true
 
  def run(self):
    global gpsd
    while gpsp.running:
      gpsd.next() #this will continue to loop and grab EACH set of gpsd info to clear the buffer

#################################################################
def displayGpsInfo():
      global latitude
      latitude=gpsd.fix.latitude
      global longitude
      longitude=gpsd.fix.longitude
      global utctime
      utctime=gpsd.utc
      global speedGPS
      speedGPS=gpsd.fix.speed
      global pointing
      pointing=gpsd.fix.track
      

def displayTable():
      print "+-----------------------------+"
      print "| Latitude |",latitude
      print "| Longitude|",longitude
      print "| UTC Time |",utctime
      print "| Speed m/s|",speedGPS
      print "| Bearing: |",pointing
      print "| ShouldPt:|",shouldPoint
      print "+-----------------------------+"
      print "| WhatToDo:|", whatToDo
      print "| Distance:|", distance
      print "| Going To:|", goingToLocation
      print "+-----------------------------+"

      port.write("\n\r+--------------------+")
      port.write("\n\r| Latitude |")
      port.write(str(latitude))
      port.write("\n\r| Longitude|")
      port.write(str(longitude))
      port.write("\n\r| UTC Time |")
      port.write( str(utctime))
      port.write("\n\r| Speed m/s|")
      port.write(str(speedGPS))
      port.write("\n\r| Bearing: |")
      port.write(str(pointing))
      port.write("\n\r| ShouldPt:|")
      port.write( str(shouldPoint))
      port.write("\n\r+---------------------+")
      port.write("\n\r| WhatToDo:|")
      port.write(str(whatToDo))
      port.write("\n\r| Distance:|")
      port.write(str(distance))
      port.write("\n\r| Going To:|")
      port.write(str(goingToLocation))
      port.write("\n\r+---------------------+")



def forward():
	print "\nEmpty"
        GPIO.output(Motor1A, GPIO.HIGH)
        GPIO.output(Motor1B, GPIO.LOW)

def backward():
        print "Empty"
def stopTheEngine():
        print "\nEmpty"
        GPIO.output(Motor1A, GPIO.LOW)
        GPIO.output(Motor1B, GPIO.LOW)

def stopRudderLOW():
        print "Empty"
def backToZero():
        print "\nEmpty"
	p.ChangeDutyCycle(7.5)
        GPIO.output(Motor2A, GPIO.LOW)
        GPIO.output(Motor2B, GPIO.LOW)

def left():
        print "\nEmpty"
        p.ChangeDutyCycle(5.0)
	GPIO.output(Motor2A, GPIO.HIGH)
        GPIO.output(Motor2B, GPIO.LOW)

def right():
        print "\nEmpty"
	p.ChangeDutyCycle(10.0)
        GPIO.output(Motor2A, GPIO.LOW)
        GPIO.output(Motor2B, GPIO.HIGH)

#########LOOP########LOOP#######LOOP####
########################################
#print "right"
#right()
#time.sleep(2)

#print "left"
#left()
#time.sleep(2)

#print "right"
#right()
#time.sleep(2)

#print "left"
#left()
#time.sleep(2)

#backToZero()

#print "forward"
#forward()
#time.sleep(2)

#print "stop"
#stopTheEngine()

#if __name__ == '__main__':
 # gpsp = GpsPoller() # create the thread
  #try:
   # gpsp.start() # start it up
    #while True:
     # displayGpsInfo()
      #multiplePoint(finalX, finalY, finalX2, finalY2, finalX3, finalY3, finalX4, finalY4, finalX5, finalY5)
      #displayTable()
      #os.system('clear')
      #time.sleep(0.1) #set to whatever
#  except (KeyboardInterrupt, SystemExit): #when you press ctrl+c
 #   print "\nKilling Thread..."
  #  gpsp.running = False
   # gpsp.join() # wait for the thread to finish what it's doing
 # print "Done.\nExiting."




calcDist(38.190254, -83.429583,38.190714, -83.430018)
#turnToWhichSide(pointing, shouldPoint) ##look the fin.dest way
#turnDegrees(x, y, a, b)
displayTable()
