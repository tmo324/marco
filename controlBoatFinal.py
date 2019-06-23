#!/usr/bin/python 
# adapted from https://github.com/recantha/EduKit3-RC-Keyboard/blob/master/rc_keyboard.py
 
import sys, termios, tty, os, time
from math import *
import pigpio
import threading
import RPi.GPIO as GPIO
from ina219 import INA219, DeviceRangeError
from rockBlock import rockBlockProtocol
#import dht11 


GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)


###############ServoMethods################
def setServo( servoNum, pulses):
        pi.set_servo_pulsewidth(servoNum, pulses)
def setServoAngle ( servoNum1, position ):
#      print ("Servo {} {} micro pulses".format(s, pw))
    pw = position * 500 / 90 + 1500    
    pi.set_servo_pulsewidth(servoNum1, pw)
pi=pigpio.pi()
#+----------------------------------------------------4.AllPins----------------$
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
#+------------------------------------------------------------
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
#+---------------------------------------------9.Navigation--------------------$
def forward():
    setServo(thrusterPin, 1600)

#def forward():
#    setServo(thrusterPin, 1650)
def speed(x):
    setServo(thrusterPin, x)

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
def backward():
    print "BACKWARDS"
    setServo(thrusterPin, 1350)
def stopEverything():
    stopTheEngine()
    straight()

#+---------------------------------------------------------
def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
 
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch
 
button_delay = 0.2
 
while True:
    char = getch()
 
    if (char == "q"):
        print("Quitting the program!")
        exit(0)
 



    if (char == "a"):
        print("LEFT  pressed <-")
        left()
        time.sleep(button_delay)
    elif (char == "d"):
        print("RIGHT pressed ->")
        right()
        time.sleep(button_delay)
    elif (char == "w"):
        print("UP pressed")
        forward()
        time.sleep(button_delay)
    elif (char == "s"):
        print("DOWN pressed")
        backward()
        time.sleep(button_delay)
 
    elif (char == "p"):
        print("Stopping everything")
        stopEverything()
        time.sleep(button_delay)
    elif (char == "o"):
        print("go straight")
        #backToZero()
        time.sleep(button_delay)


    elif (char == "1"):
        print("Speed:1550 ")
        speed(1550)
        time.sleep(button_delay)
    elif (char == "2"):
        print("Speed:1600 ")
        speed(1600)
        time.sleep(button_delay)
    elif (char == "3"):
        print("Speed:1650 ")
        speed(1650)
        time.sleep(button_delay)
    elif (char == "4"):
        print("Speed:1700 ")
        speed(1700)
        time.sleep(button_delay)
    elif (char == "5"):
        print("Speed:1750 ")
        speed(1750)
        time.sleep(button_delay)
    elif (char == "6"):
        print("Speed:1800 ")
        speed(1800)
        time.sleep(button_delay)
