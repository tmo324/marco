#!/usr/bin/python
# -*- coding: utf-8 -*-

#MARCO Boat V1.0 Final Program. This is the final version of the program of MARCO Boat V1.0.
#It includes autonomous navigating, temperature sensing, gyroscoping, and current sensing.
#It doesn't include LoRa communication.
#Created by Tergel Molom-Ochir for Project MARCO. Updated on Wednesday 30th, August 2018. 

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
#7a.ROCKBlock
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
import rf95
import math
import subprocess
import rockBlock
from math import *
import pigpio
import threading
import RPi.GPIO as GPIO
from ina219 import INA219, DeviceRangeError
from rockBlock import rockBlockProtocol
#import dht11 


#+---------------------------------------------2.ThingsToSetUp------------------------------------------------+
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
gpsd = None #seting the global variable

#INA219 Sensor
SHUNT_OHMS = 0.1
MAX_EXPECTED_AMPS = 2.0
ina = INA219(SHUNT_OHMS, MAX_EXPECTED_AMPS)
ina.configure(ina.RANGE_16V)

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
shortTime=0

voltageReport=0 #String from Voltage of the 12V battery. This can be used for speed adjustment.
batteryVoltage=0 #Voltage of the 12V battery
systemCurrent=0 #current of the whole system

timesDone=0 #how many laps the boat has completed
piTemp=0 #CPU temperature of the Pi

i=0 #counter for the loop. every i seconds, the program sends message.

#Probes
probe1=0
probe2=0
probe3=0
avgProbe=0 #average temperature from the sensors.
probeSum=0 #total sum of the temperatures from probes.
probeNumber=0 #from how many sensors are we getting response?

xrotation=0 #for gyroscope. front and back.
yrotation=0 #for gyroscope. sideways.

waypoint=[0]

strMessage=''

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
#thrusterPin=18 #GPIO18 OLD
thrusterPin=24 #GPIO24 NEW moved bc had to install ROCKBlock
servoPin=23    #GPIO23

#Thruster
#setServo(thrusterPin, 1500) #pin 12 on board (GPIO18 PCM_CLK)
setServo(thrusterPin, 1500) #pin 18 on board (GPIO24 PCM_CLK)
#14Board for Ground

#Servo
setServoAngle(servoPin, -15) #0degrees

#DHT11
#instance = dht11.DHT11(pin=24)
#result = instance.read()

##LoRa
##lora = rf95.RF95(CS, INT(G0/IRQ), RST)
#lora = rf95.RF95(0, 25, 17) #GPIO original
#Bw125Cr45Sf128 = (0x72,   0x74,    0x00)
##lora.cleanup()
#if not lora.init(): # returns True if found
#    print("RF95 not found")
#    lora.cleanup()
#    quit(1)
#else:
#    print("RF95 LoRa mode ok")
#lora.set_frequency(433)
#lora.set_tx_power(23)
#lora.send([0x00, 0x01, 0x02, 0x03])
#lora.wait_packet_sent()
#
#header=[0xff, 0xff, 0, 0]

#+------------------------------------------------------5.Setup-------------------------------------------------------+
#https://www.darrinward.com/lat-long/?id=5a76032f24a6e2.23164443

def prepLocations():
    f = open('/home/pi/programs/actualPrograms/autonomousBoat/coordinates.txt')
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
    global goingToLocation
    global printedCount
    global count
    global timesDone
    if count == 1:
        fromPointToPoint(latitude, longitude, a1, b1)
        #global goingToLocation
	goingToLocation='1st'
	#global printedCount
        printedCount=count
        #print '->1st'
    elif count == 2:
        fromPointToPoint(latitude, longitude, a2, b2)
        #global goingToLocation
        goingToLocation='2nd'
        #global printedCount
        printedCount=count
	#print '->2nd'
    elif count == 3:
        fromPointToPoint(latitude, longitude, a3, b3)
        #global goingToLocation
        goingToLocation='3rd'
        #global printedCount
        printedCount=count
	#print '->3rd'
    elif count == 4:
        fromPointToPoint(latitude, longitude, a4, b4)
        #print '->4th'
        #global goingToLocation
        goingToLocation='4th'
        #global printedCount
        printedCount=count
    elif count == 5:
        fromPointToPoint(latitude, longitude, a5, b5)
        #print '->5th'
        #global goingToLocation
        goingToLocation='5th'
        #global printedCount
        printedCount=count
        
        #global count
        count=1
        #global timesDone
        timesDone+=1
    else:
        #global goingToLocation
        goingToLocation='REACHED'
        stopEverything()

def fromPointToPoint(x, y, a, b):
    global count
    global whatToDo
    if (x < a + 0.0000149 and x > a - 0.0000149 and y < b + 0.0000149 and y > b - 0.0000149): #is the boat at the fin.dest?
	#global count
        count+=1
    else:
        turnDegrees(x, y, a, b)#calculates/thinks where the car should point
        calcDist(x, y, a, b)
        if (pointing > shouldPoint - 20 and pointing < shouldPoint + 20 ): ##is it already pointing at that point?
            forward()
            straight()
            #global whatToDo
            whatToDo=("Matching/JustGoFwd")
        else:
            turnToWhichSide(pointing, shouldPoint) ##look the fin.dest way
            #global whatToDo
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
    global distance
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

    #global distance
    distance=dist

def measure_Pi_temp():
    temp = os.popen("vcgencmd measure_temp").readline()
    global piTemp
    piTemp=temp
    global piTemp
    piTemp=(piTemp.replace("temp=",""))
    global piTemp
    piTemp=(piTemp.replace("'C",""))
    return piTemp

def measure_accelerometer():
    gyro_xout = read_word_2c(0x43)
    gyro_yout = read_word_2c(0x45)
    gyro_zout = read_word_2c(0x47)
    accel_xout = read_word_2c(0x3b)
    accel_yout = read_word_2c(0x3d)
    accel_zout = read_word_2c(0x3f)
    accel_xout_scaled = accel_xout / 16384.0
    accel_yout_scaled = accel_yout / 16384.0
    accel_zout_scaled = accel_zout / 16384.0
    #print "x rotation: " , get_x_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled)
    #print "y rotation: " , get_y_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled)
    global xrotation
    xrotation=get_x_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled)
    global yrotation
    yrotation=get_y_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled)

#Methods for Temperature Probes
def measureTempProbes():
    try:
        probe1=subprocess.check_output('cat /sys/bus/w1/devices/28-011833b210ff/w1_slave | grep "t=" | awk \'{print $10}\' | awk -F= \'{print $2}\' 2> /dev/null',shell=True)
    except:
        print ("")
    try:
        probe2=subprocess.check_output('cat /sys/bus/w1/devices/28-011833b2f7ff/w1_slave | grep "t=" | awk \'{print $10}\' | awk -F= \'{print $2}\' 2> /dev/null',shell=True)
    except:
        print ("")
    try:
        probe3=subprocess.check_output('cat /sys/bus/w1/devices/28-011833b4c7ff/w1_slave | grep "t=" | awk \'{print $10}\' | awk -F= \'{print $2}\' 2> /dev/null',shell=True)
    except:
        print ("")
    check (probe1)
    check (probe2)
    check (probe3)
    global avgProbe
    if (probeNumber > 0):
        avgProbe=round(((probeSum)/probeNumber), 1)
    else:
        avgProbe=0.0
    print ("Average Temperature is %4.2f" % avgProbe)

def check(temp):
    global probeSum
    global probeNumber
    if (temp != ""):
        probeSum = probeSum + float(temp)/1000.0
        probeNumber = probeNumber + 1

def updateBatteryReport():
    global voltageReport
    if (batteryVoltage>12.5):
        #global voltageReport
        voltageReport=90
        print "90%"
    elif (12.5>batteryVoltage and batteryVoltage>12.4):
        #global voltageReport
        voltageReport=8090
        print "80-90%"
    elif (12.4>batteryVoltage and batteryVoltage>12.3):
        #global voltageReport
        voltageReport=7080
        print "70-80%. Stop Running motor now!"
    elif (12.3>batteryVoltage and batteryVoltage>12.20):
        #global voltageReport
        voltageReport=6070
        print "60-70%"
    elif (12.20>batteryVoltage and batteryVoltage>12.1):
        #global voltageReport
        voltageReport=5060
        print "50-60%"
    elif (12.1>batteryVoltage):
        #global voltageReport
        voltageReport=050
        print "Less than 50% to run!"
#        print "Going to sleep"
#        MoExample().main("Battery too low!Shutting down.")
#        #call("sudo shutdown -h now", shell=True)
#        os.system("sudo shutdown -r now")

def updateInfo():
    global latitude
    latitude=gpsd.fix.latitude
    global longitude
    longitude=gpsd.fix.longitude
    global utctime
    utctime=gpsd.utc
    global shortTime
    shortTime=utctime[2:4]+utctime[5:7]+utctime[8:10]+utctime[11:13]+utctime[14:16]+utctime[17:19]
    global speedGPS
    speedGPS=gpsd.fix.speed
    global pointing
    pointing=gpsd.fix.track
    global batteryVoltage
    batteryVoltage=ina.voltage()+0.2
    print("Battery Voltage is:" + str(batteryVoltage))
    global systemCurrent
    systemCurrent=ina.current()/1000
    measure_Pi_temp() 
#    measure_accelerometer() #xrotation and yrotation get updated inside this definition

def makeAndSendString(): 
    #str=lat,lon,time,bearing,shouldPoint,whatToDo,distance,timesDone,goingToLocation,ina.voltage(),result.temperature
    #strMessage=str(latitude) + "," + str(longitude) + "," + str(utctime) + "," + str(pointing) + "," + str(shouldPoint) + "," + str(whatToDo) + "," + str(distance)+","+str(timesDone) + "," + str(round(batteryVoltage, 1)) + "," + str(goingToLocation) +"," + str(printedCount) +"," + str(piTemp) + ","+ str(round(xrotation, 1)) + "," + str(round(yrotation, 1)) + "," + str(avgProbe)
    global strMessage
    strMessage=str(round(latitude,5)) + "," + str(round(longitude,5)) + "," + str(shortTime) + "," + str(round(pointing, 1)) + "," + str(round(speedGPS, 1)) + "," + str(int(round(distance))) + "," + str(round(batteryVoltage, 1)) + "," + str(round(systemCurrent, 1)) + "," + str(goingToLocation) +"," + str(round(xrotation, 1)) + "," + str(round(yrotation, 1)) + "," + str(avgProbe)+ "," +str(piTemp)
    #MoExample().main(str(strMessage))
    file=open("/home/pi/programs/actualPrograms/autonomousBoat/messages.txt", "a")
    file.write(str(strMessage))
    file.close()
    print(strMessage)

def sendIridium():
    MoExample().main(str(strMessage))
    file=open("/home/pi/programs/actualPrograms/autonomousBoat/messages.txt", "a")
    file.write("Just sent msg to Iridium")
    file.close()


#def sendLora(abc):
#    lora.send(header+lora.str_to_data(abc+"\0"))
#    lora.wait_packet_sent()

#+----------------------------------------------------7.Accelerometer-----------------------------------------------------+
def read_byte(adr):
    return bus.read_byte_data(address, adr)
def read_word(adr):
    high = bus.read_byte_data(address, adr)
    low = bus.read_byte_data(address, adr+1)
    val = (high << 8) + low
    return val
def read_word_2c(adr):
    val = read_word(adr)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val
def dist(a,b):
    return math.sqrt((a*a)+(b*b))
def get_y_rotation(x,y,z):
    radians = math.atan2(x, dist(y,z))
    return -math.degrees(radians)
def get_x_rotation(x,y,z):
    radians = math.atan2(y, dist(x,z))
    return math.degrees(radians)

#+----------------------------------------------------7a.ROCKBlock-----------------------------------------------------+
class MoExample (rockBlockProtocol):
    
    def main(self, str):
      
        rb = rockBlock.rockBlock("/dev/ttyUSB0", self)
        
        rb.sendMessage(str)      
        
        rb.close()
        
    def rockBlockTxStarted(self):
        print "rockBlockTxStarted"
        
    def rockBlockTxFailed(self):
        print "rockBlockTxFailed"
        
    def rockBlockTxSuccess(self,momsn):
        print "rockBlockTxSuccess " + str(momsn)
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
#+---------------------------------------------9a.IsBoatAtTheLake-----------------------------------------------+
def IsBoatAtTheLake(x, y):
    a=38.195365
    b=-83.435316
    if (x < a + 0.003 and x > a - 0.003 and y < b + 0.003 and y > b - 0.003): #is the boat at the fin.dest?
        #global count
        print "Yes,it's at the lake. Safe."
    else:
        print "WARNING:Boat not at the lake!"
        #sendIridium(str("Not lake,")+str(latitude)+str(",")+str(longitude))
        MoExample().main(str("Not lake"))

#+---------------------------------------------9.Navigation-----------------------------------------------+
def forward():#
    updateBatteryReport()#
    if(voltageReport==90):
        setServo(thrusterPin, 1700)
    elif(voltageReport==8090):
        setServo(thrusterPin, 1650)
    elif(voltageReport==7080):
        setServo(thrusterPin, 1500)
    elif(voltageReport==6070):
        setServo(thrusterPin, 1500)
    elif(voltageReport==5060):
        setServo(thrusterPin, 1500)
    elif(voltageReport==050):
        setServo(thrusterPin, 1500)

#def forward():
#    setServo(thrusterPin, 1650)

def stopTheEngine():
    setServo(thrusterPin, 1500)

def straight():
    print "straight"
    setServoAngle(servoPin, -60)
    time.sleep(1)
    pi.set_servo_pulsewidth(servoPin, 0)

def left():
    print "left"
    setServoAngle(servoPin, 20)
    time.sleep(1)
    pi.set_servo_pulsewidth(servoPin, 0)

def right():
    print "right"
    setServoAngle(servoPin, -120)
    time.sleep(1)
    pi.set_servo_pulsewidth(servoPin, 0)

def stopEverything():
    stopTheEngine()
    straight()




#+---------------------------------------------10.Loop----------------------------------------------------+
print "Starting now"
prepLocations()
stopEverything()

right()
left()
straight()
stopEverything()


print "about to edit a file"
file=open("/home/pi/logfile.txt", "a")
file.write("\n")
file.write("Starting now!")
file.close()
print "STARTING NOW -> /home/pi/logfile.txt"

if __name__ == '__main__':
    gpsp = GpsPoller() # create the thread
    try:
       global i
       gpsp.start() # start it up
       makeAndSendString()
       #sendIridium()

       print "LOOP START NOW!"
       while True:
            print "----------------------------------------------"
            updateInfo()
            multiplePoint(finalX, finalY, finalX2, finalY2, finalX3, finalY3, finalX4, finalY4, finalX5, finalY5)
            #IsBoatAtTheLake(latitude, longitude)
            i=i+1
            print i
            makeAndSendString()
            if i%60==0:
                #measureTempProbes()
                makeAndSendString()
                #sendIridium()
                print "Just makeAndSendString"
                #global i
                i=i+3
                print i
                time.sleep(3)
                print "3sec delay"
            if i%72000==0:
                print "send to iridium"
                sendIridium()
            file=open("/home/pi/logfile.txt", "a")
            file.write("\n")
            file.write(str(i))
            file.close()

            #os.system('clear')
#            time.sleep(1) #set to whatever
    except (KeyboardInterrupt, SystemExit): #when you press ctrl+c
        print "\nKilling Thread..."
        pi.stop()
#        file.close() 
#        lora.cleanup()
        gpsp.running = False
        gpsp.join() # wait for the thread to finish what it's doing
    print "Done.\nExiting."

