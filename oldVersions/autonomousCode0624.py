# -*- coding: utf-8 -*-

#+---------------------------------------------Imports------------------------------------------------+
#####REMINDER: GPIO MODE BCM
import os
from gps import *
from time import *
import time
import rf95
import math
from math import *
import pigpio
import threading
import RPi.GPIO as GPIO
from ina219 import INA219, DeviceRangeError
#import dht11 

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
gpsd = None #seting the global variable

SHUNT_OHMS = 0.1
MAX_EXPECTED_AMPS = 2.0
ina = INA219(SHUNT_OHMS, MAX_EXPECTED_AMPS)
ina.configure(ina.RANGE_16V)



##############Serial####################
import serial
port = serial.Serial("/dev/ttyAMA0", baudrate=57600, timeout=0.5)

###############Variables################
count=1
shouldPoint=0
whatToDo=0
distance=0
goingToLocation=0

latitude=0
longitude=0
utctime=0
speedGPS=0
pointing=0

voltageReport=0
batteryVoltage=0
timesDone=0
###############ServoMethods################
def setServo( servoNum, pulses):
        pi.set_servo_pulsewidth(servoNum, pulses)
def setServoAngle ( servoNum1, position ):
#      print ("Servo {} {} micro pulses".format(s, pw))
    pw = position * 500 / 90 + 1500    
    pi.set_servo_pulsewidth(servoNum1, pw)
pi=pigpio.pi()
#+---------------------------------------------AllPins--------------------------------------------------+
#BCMMODE GPIOMODE
thrusterPin=18 #GPIO18
servoPin=23    #GPIO23

#LoRa
lora = rf95.RF95(0, 25, 17) #GPIO
Bw125Cr45Sf128 = (0x72,   0x74,    0x00)
lora.set_frequency(433)
lora.set_tx_power(23)
header=[0xff, 0xff, 0, 0]

#Thruster
setServo(thrusterPin, 1500) #pin 12 on board (GPIO18 PCM_CLK)
#14Board for Ground

#Servo
setServoAngle(servoPin, -15) #0degrees

#Antenna
#2, 6, 8(TX), and 10(RX)(All Board) 

#DHT11
#instance = dht11.DHT11(pin=24)
#result = instance.read()
#+---------------------------------------------Setup---------------------------------------------------------+
#https://www.darrinward.com/lat-long/?id=5a76032f24a6e2.23164443

#Eagle Lake Long Test
waypoint = [38.193670, -83.435529,
38.194851, -83.435336,
38.196756, -83.436109,
38.195104, -83.434918,
38.193008, -83.434801]

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

#+---------------------------------------------MethodsBeginHere---------------------------------------------------+
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
        global count
        count=1
        global timesDone
        timesDone+=1
    else:
        global goingToLocation
        goingToLocation='REACHED'
        stopEverything()


def fromPointToPoint(x, y, a, b):
    if (x < a + 0.0000099 and x > a - 0.0000099 and y < b + 0.0000099 and y > b - 0.0000099): #is the boat at the fin.dest?
	global count
        count+=1
    else:
        turnDegrees(x, y, a, b)#calculates/thinks where the car should point

        calcDist(x, y, a, b)
        if (pointing > shouldPoint - 20 and pointing < shouldPoint + 20 ): ##is it already pointing at that point?
            forward()
            backToZero()
            global whatToDo
            whatToDo=("Matching/Just go fwd \n")
        else:
            turnToWhichSide(pointing, shouldPoint) ##look the fin.dest way
            global whatToDo
            whatToDo=("Need to turn \n")

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
            port.write("Right")
            print "Need to Right"
        else: 
            left()
            forward()
            port.write("Left")
            print "Need to Left"
    else:
        if (x-180<a and a<x):
            left()
            forward()
            port.write("Left")
            print "Need to Left"
        else:
            right()
            forward()
            port.write("Right")
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
def updateBatteryReport():
      if (batteryVoltage>12.5):
            global voltageReport
            voltageReport=90
            print "90%"
      elif (12.5>batteryVoltage and batteryVoltage>12.42):
            global voltageReport
            voltageReport=8090
            print "80-90%"
      elif (12.42>batteryVoltage and batteryVoltage>12.32):
            global voltageReport
            voltageReport=7080
            print "70-80%"
      elif (12.32>batteryVoltage and batteryVoltage>12.20):
            global voltageReport
            voltageReport=6070
            print "60-70%"
      elif (12.20>batteryVoltage and batteryVoltage>12.06):
            global voltageReport
            voltageReport=5060
            print "50-60%"
      elif (12.06>batteryVoltage):
            global voltageReport
            voltageReport=050
            print "Less than 50% to run!"

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
      global batteryVoltage
      batteryVoltage=ina.voltage()
      print("Battery Voltage is:" + str(batteryVoltage))

def makeAndSendString(): #str=lat,lon,time,bearing,shouldPoint,whatToDo,distance,timesDone,goingToLocation,ina.voltage(),result.temperature
      #strMessage=str(latitude) + "," + str(longitude) + "," + str(utctime) + "," + str(pointing) + "," + str(shouldPoint) + "," + str(whatToDo) + "," + str(distance)+","+str(timesDone) + "," + str(goingToLocation) + "," + str(ina.voltage())
      #print (strMessage)
      print ("/makeAndSendString/ About to sendLora")
      #sendLora(str(strMessage))
      sendLora(str("Je"))
      print ("I got here on the next page")


def sendLora(abc):
      print("/sendLora About to start")
      lora.send(header+lora.str_to_data(abc+"\0"))
      print("Just sent stuff")
      lora.wait_packet_sent()
      print("Just sent stuff222")


def displayTable():
      port.write("\n\r+----------------------------------+")
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
      port.write("\n\r+-----------------------------------+")
      port.write("\n\r| WhatToDo:|")
      port.write(str(whatToDo))
      port.write("\n\r| Distance:|")
      port.write(str(distance))
      port.write("\n\r| Going To:|")
      port.write(str(goingToLocation))
      port.write("\n\r+------Meter Data-------------------+")
      port.write("\n\r| Battery Voltage:|")
      port.write(str('{0:0.2f}V'.format(ina.voltage())))
      port.write("\n\r| Bus Current:|")
      port.write(str('{0:0.2f}mA'.format(ina.current())))
      port.write("\n\r| Power      :|")
      port.write(str('{0:0.2f}mW'.format(ina.power())))
      port.write("\n\r|ShuntVoltage:|")
      port.write(str('{0:0.2f}mV'.format(ina.shunt_voltage())))
      port.write("\n\r+----------ESC-Info--------------------+")
      #port.write("\n\r| ESC Temp :|")
      #port.write(str(result.temperature))
      #port.write("\n\r| ESC Humid:|")
      #port.write(str(result.humidity))
      port.write("\n\r+-----------------------------------+")

#+---------------------------------------------Navigation-----------------------------------------------+
def forward():
	updateBatteryReport()
	if(voltageReport==90):
		setServo(thrusterPin, 1650)
	elif(voltageReport==8090):
		setServo(thrusterPin, 1650)
	elif(voltageReport==7080):
		setServo(thrusterPin, 1600)
	elif(voltageReport==6070):
		setServo(thrusterPin, 1570)
	elif(voltageReport==5060):
		setServo(thrusterPin, 1500)
	elif(voltageReport==050):
		setServo(thrusterPin, 1500)
def speed(x):
	setServo(thrusterPin, x)

def stopTheEngine():
	setServo(thrusterPin, 1500)

def backToZero():
        setServoAngle(servoPin, -15)

def left():
        setServoAngle(servoPin, 90)

def right():
        setServoAngle(servoPin, -120)


def stopEverything():
	stopTheEngine()
	backToZero()

#+---------------------------------------------Loop----------------------------------------------------+
displayTable()
#forward()
#time.sleep(1)
#stopEverything()

#backToZero()
#time.sleep(60)

#print "right"
#right()
#time.sleep(1)

#print "left"
#left()
#time.sleep(1)

#backToZero()
#time.sleep(1)

#print "forward"
#forward()
#time.sleep(1)

#stopEverything()



#if __name__ == '__main__':
#  gpsp = GpsPoller() # create the thread
#  try:
#    gpsp.start() # start it up
#    while True:
#      updateInfo()
#      multiplePoint(finalX, finalY, finalX2, finalY2, finalX3, finalY3, finalX4, #finalY4, finalX5, finalY5)
#      displayTable()
#      makeAndSendString()
#      print("just printed makeAndSendString")
#      #os.system('clear')
#      time.sleep(0.1) #set to whatever
#  except (KeyboardInterrupt, SystemExit): #when you press ctrl+c
#    print "\nKilling Thread..."
#    pi.stop()
#    gpsp.running = False
#    gpsp.join() # wait for the thread to finish what it's doing
#  print "Done.\nExiting."





gpsp = GpsPoller() # create the thread
gpsp.start() # start it up
try:
  while True:
    print("******NEW******WEWENNEWNEW***WENNWE*EW*EN")
    #updateInfo()
    #multiplePoint(finalX, finalY, finalX2, finalY2, finalX3, finalY3, finalX4, finalY4, finalX5, finalY5)
    #displayTable()
    print("About to go Make and send string")
    
    makeAndSendString()
    print("just printed makeAndSendString")
    #os.system('clear')
    time.sleep(0.1) #set to whatever
except (KeyboardInterrupt, SystemExit): #when you press ctrl+c
  print "\nKilling Thread..."
  pi.stop()
  gpsp.running = False
  gpsp.join() # wait for the thread to finish what it's doing
print "Done.\nExiting."

