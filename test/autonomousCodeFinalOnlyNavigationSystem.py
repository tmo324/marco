#MARCO Boat V1.0 Navigation System Test.
#Created by Tergel Molom-Ochir for MARCO navigation system's test.  

#!/usr/bin/python
# -*- coding: utf-8 -*-
#from __future__ import print_function

#Guide to this file
#1.Imports
#2.ThingsToSetUp
#3.Variables
#4.AllPins
#5.Setup
#6.MethodsBeginHere
#7.Accelerometer
#8.GPS
#9.Navigation
#10.Loop

#+---------------------------------------------------1.Imports------------------------------------------------+
#####REMINDER: GPIO MODE BCM
import os
from gps import *
from time import *
import time
import smbus
#import rf95
import math
import subprocess 
from math import *
import pigpio
import threading
import RPi.GPIO as GPIO
#from ina219 import INA219, DeviceRangeError
#import dht11 

#+---------------------------------------------2.ThingsToSetUp------------------------------------------------+
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
gpsd = None #seting the global variable

##INA219 Sensor
#SHUNT_OHMS = 0.1
#MAX_EXPECTED_AMPS = 2.0
#ina = INA219(SHUNT_OHMS, MAX_EXPECTED_AMPS)
#ina.configure(ina.RANGE_16V)

##Accelerometer Power management registers
#power_mgmt_1 = 0x6b
#power_mgmt_2 = 0x6c
#bus = smbus.SMBus(1) # or bus = smbus.SMBus(1) for Revision 2 boards
#address = 0x68       # This is the address value read via the i2cdetect command
## Now wake the 6050 up as it starts in sleep mode
#bus.write_byte_data(address, power_mgmt_1, 0)

#+----------------------------------------------------3.Variables------------------------------------------------+
count=1
shouldPoint=0 #the boat should be pointing to this to get to the next location
whatToDo=0 #what should the boat do next?
distance=0 #how far is the boat from the next locatiuon
goingToLocation=0 #which location is the boat going to?

latitude=0 #
longitude=0 #
utctime=0
speedGPS=0 #speed in maybe m/s?
pointing=0 #boat's current exact location.
printedCount=0 #which waypoint is the boat going to

#voltageReport=0 #String from Voltage of the 12V battery. This can be used for speed adjustment.
#batteryVoltage=0 #Voltage of the 12V battery
#systemCurrent=0 #current of the whole system

timesDone=0 #how many laps the boat has completed
piTemp=0 #CPU temperature of the Pi

i=0 #counter for the loop. every i seconds, the program sends message.

##Probes
#probe1=0
#probe2=0
#probe3=0
#avgProbe=0 #average temperature from the sensors.
#probeSum=0 #total sum of the temperatures from probes.
#probeNumber=0 #from how many sensors are we getting response?

#xrotation=0 #for gyroscope. front and back.
#yrotation=0 #for gyroscope. sideways.

waypoint=[0]

finalX = 0
finalY = 0
finalX2 = 0
finalY2 = 0
finalX3 = 0
finalY3 = 0
finalX4 = 0
finalY4 = 0
finalX5 = 0
finalY5=0
#file=open("messages.txt", "w")

###############ServoMethods################
def setServo( servoNum, pulses):
        pi.set_servo_pulsewidth(servoNum, pulses)
def setServoAngle ( servoNum1, position ):
#      print ("Servo {} {} micro pulses".format(s, pw))
    pw = position * 500 / 90 + 1500    
    pi.set_servo_pulsewidth(servoNum1, pw)
pi=pigpio.pi()


#+----------------------------------------------------4.AllPins--------------------------------------------------+
#BCMMODE GPIOMODE
servoPin=23    #GPIO23
GPIO.setup(24,GPIO.OUT)

#Servo
setServoAngle(servoPin, -15) #0degrees

#+------------------------------------------------------5.Setup-------------------------------------------------------+
#https://www.darrinward.com/lat-long/?id=5a76032f24a6e2.23164443

def prepLocations():
    f = open('/home/pi/programs/actualPrograms/autonomousBoat/test/coordinates.txt')
    message = f.read()
    message2 = message.splitlines()
    lines = []
    for row in message2:
        lines = lines + [row.split(',')] 
    #f.close()
    global waypoint
    waypoint.remove(0)
    waypoint.append(lines[0][0])
    waypoint.append(lines[0][1])
    waypoint.append(lines[1][0])
    waypoint.append(lines[1][1])
    waypoint.append(lines[2][0])
    waypoint.append(lines[2][1])
    waypoint.append(lines[3][0])
    waypoint.append(lines[3][1])
    waypoint.append(lines[4][0])
    waypoint.append(lines[4][1])
    ##WAYPOINTS
    global finalX
    global finalY
    global finalX2
    global finalY2
    global finalX3
    global finalY3
    global finalX4
    global finalY4
    global finalX5
    global finalY5
    finalX = float(waypoint[0])
    finalY = float(waypoint[1])
    finalX2 = float(waypoint[2])
    finalY2 = float(waypoint[3])
    finalX3 = float(waypoint[4])
    finalY3 = float(waypoint[5])
    finalX4 = float(waypoint[6])
    finalY4 = float(waypoint[7])
    finalX5 = float(waypoint[8])
    finalY5 = float(waypoint[9])
    f.close()

#waypoint=[38.193049, -83.434841,
#38.193257, -83.434428,
#38.193377, -83.433782, 
#38.193501, -83.434116,
#38.193255, -83.434494]

##WAYPOINTS
#finalX = waypoint[0]
#finalY = waypoint[1]
#finalX2 = waypoint[2]
#finalY2 = waypoint[3]
#finalX3 = waypoint[4]
#finalY3 = waypoint[5]
#finalX4 = waypoint[6]
#finalY4 = waypoint[7]
#finalX5 = waypoint[8]
#finalY5 = waypoint[9]

#+--------------------------------------------------6.MethodsBeginHere-----------------------------------------------+
def multiplePoint(a1,b1,a2,b2,a3,b3,a4,b4,a5,b5):
    if count == 1:
        fromPointToPoint(latitude, longitude, a1, b1)
        global goingToLocation
	goingToLocation='1st'
	global printedCount
        printedCount=count
        #print '->1st'
    elif count == 2:
        fromPointToPoint(latitude, longitude, a2, b2)
        global goingToLocation
        goingToLocation='2nd'
        global printedCount
        printedCount=count
	#print '->2nd'
    elif count == 3:
        fromPointToPoint(latitude, longitude, a3, b3)
        global goingToLocation
        goingToLocation='3rd'
        global printedCount
        printedCount=count
	#print '->3rd'
    elif count == 4:
        fromPointToPoint(latitude, longitude, a4, b4)
        #print '->4th'
        global goingToLocation
        goingToLocation='4th'
        global printedCount
        printedCount=count
    elif count == 5:
        fromPointToPoint(latitude, longitude, a5, b5)
        #print '->5th'
        global goingToLocation
        goingToLocation='5th'
        global printedCount
        printedCount=count
        
        global count
        count=1
        global timesDone
        timesDone+=1
    else:
        global goingToLocation
        goingToLocation='REACHED'
        stopEverything()

def fromPointToPoint(x, y, a, b):
    if (x < a + 0.0000149 and x > a - 0.0000149 and y < b + 0.0000149 and y > b - 0.0000149): #is the boat at the fin.dest?
	global count
        count+=1
    else:
        turnDegrees(x, y, a, b)#calculates/thinks where the car should point
        calcDist(x, y, a, b)
        if (pointing > shouldPoint - 20 and pointing < shouldPoint + 20 ): ##is it already pointing at that point?
            forward()
            straight()
            global whatToDo
            whatToDo=("Matching/JustGoFwd")
        else:
            turnToWhichSide(pointing, shouldPoint) ##look the fin.dest way
            global whatToDo
            whatToDo=("NeedToTurn")

def calcBearing(lat1, lon1, lat2, lon2):
    dLon = lon2 - lon1
    y = sin(dLon) * cos(lat2)
    x = cos(lat1) * sin(lat2)  - sin(lat1) * cos(lat2) * cos(dLon)
    return atan2(y, x)
def turnDegrees(x, y, a, b):
    Bearing = calcBearing(x,y,   a,b)
    Bearing = degrees(Bearing)
    Bearing = (Bearing + 360) % 360
    global shouldPoint
    shouldPoint=Bearing
def turnToWhichSide( x, a ): #calculates turning to which side is the shortest
    if(x<180):
        if (x<a and a<x+180):
            right()
            forward()
            #port.write("Right")
            print "Need to Right"
        else: 
            left()
            forward()
            #port.write("Left")
            print "Need to Left"
    else:
        if (x-180<a and a<x):
            left()
            forward()
            #port.write("Left")
            print "Need to Left"
        else:
            right()
            forward()
            #port.write("Right")
            print "Need to Right"

def calcDist( x,  y,  a,  b):
    num1 =  (math.fabs(x)) #long
    num2 =  (math.fabs(y)) #lat
    num3 =  (math.fabs(a)) #long
    num4 =  (math.fabs(b)) #lat

    #Each degree of  latitudeX is approximately 69 miles=111.045 kilometers=4,371,840 Inches=4,371,850.393701 Inches=364320ft
    #Each degree of longitudeY is approximately 69.172 miles=111.321543 kilometers=4382737.913386inches=365228.16ft
    xDiff =  (math.fabs(num1 - num3)) #long
    yDiff =  (math.fabs(num2 - num4)) #lat
    #now to inches
    xDiff = xDiff * 364320
    yDiff = yDiff * 365228.16
    dist = math.sqrt(xDiff * xDiff + yDiff * yDiff)

    global distance
    distance=dist

def measure_Pi_temp():
    temp = os.popen("vcgencmd measure_temp").readline()
    global piTemp
    piTemp=temp
    return (temp.replace("",""))

def updateInfo():
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
    measure_Pi_temp() 

def makeAndSendString(): 
    strMessage=str(latitude) + "," + str(longitude) + "," + str(utctime) + "," + str(pointing)+ "," + str(shouldPoint) + "," + str(speedGPS) + "," + str(distance)+ "," + str(goingToLocation) + "," + str(timesDone)+"," + str(whatToDo)
    file=open("forTest.txt", "a")
    file.write("\n")
    file.write(str(strMessage))
    file.close()
    print(strMessage)

def dist(a,b):
    return math.sqrt((a*a)+(b*b))

#+-------------------------------------------------------8.GPS-----------------------------------------------------------+
class GpsPoller(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        global gpsd #bring it in scope
        gpsd = gps(mode=WATCH_ENABLE) #starting the stream of info
        self.current_value = None
        self.running = True #setting the thread running to true
    def run(self):
        global gpsd
        while gpsp.running:                                                                                          #
            gpsd.next() #this will continue to loop and grab EACH set of gpsd info to clear the buffer  

#+---------------------------------------------9.Navigation-----------------------------------------------+

def forward():
    GPIO.output(24,GPIO.HIGH)
    print "going forward"

def stopTheEngine():
    GPIO.output(24,GPIO.OUT)

def straight():
    print "straight"
    setServoAngle(servoPin, -60)
    time.sleep(1)
    pi.set_servo_pulsewidth(servoPin, 0)

def left():
    print "left"
    setServoAngle(servoPin, -120)
    time.sleep(1)
    pi.set_servo_pulsewidth(servoPin, 0)

def right():
    print "right"
    setServoAngle(servoPin, 20)
    time.sleep(1)
    pi.set_servo_pulsewidth(servoPin, 0)

def stopEverything():
    stopTheEngine()
    straight()

#+---------------------------------------------10.Loop----------------------------------------------------+
prepLocations()
#print "A Little Test. Warm up"
stopEverything()
forward()
right()
left()
stopEverything()

print "5"
time.sleep(5)
print "5 sec now"

forward()
right()
left()
stopEverything()


asdf=open("/home/pi/logfile.txt", "a")
asdf.write("\n")
asdf.write("Starting now!")
asdf.close()

if __name__ == '__main__':
    gpsp = GpsPoller() # create the thread
    try:
        gpsp.start() # start it up
        while True:
            updateInfo()
            multiplePoint(finalX, finalY, finalX2, finalY2, finalX3, finalY3, finalX4, finalY4, finalX5, finalY5)
            i=i+1
            if i%2==0:
                #measureTempProbes()
                makeAndSendString()
                time.sleep(3)
            os.system('clear')
#            time.sleep(1) #set to whatever
            asdf=open("/home/pi/logfile.txt", "a")
            asdf.write("\n")
            asdf.write(str(i))
            asdf.close()


    except (KeyboardInterrupt, SystemExit): #when you press ctrl+c
        print "\nKilling Thread..."
        pi.stop()
#        file.close() 
        gpsp.running = False
        gpsp.join() # wait for the thread to finish what it's doing
    print "Done.\nExiting."
