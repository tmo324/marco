#!/usr/bin/python3
from __future__ import print_function

import time
import rf95
import RPi.GPIO as GPIO
GPIO.setwarnings(True)


file=open("/home/pi/logfile.txt", "a")
file.write("\n")
file.write("A")
file.close()
# Create rf95 object with CS0 and external interrupt on pin 25
#lora = rf95.RF95(CS, INT(G0/IRQ), RST)
#lora = rf95.RF95(0, 25, 17) #GPIO original
lora = rf95.RF95(0, 25) #GPIO original

#lora = rf95.RF95(0, 24, 4) #GPIO
#lora = rf95.RF95(0, 17, 22) #GPIO test
Bw125Cr45Sf128 = (0x72,   0x74,    0x00)

file=open("/home/pi/logfile.txt", "a")
file.write("\n")
file.write("B")
file.close()

if not lora.init(): # returns True if found
    print("RF95 not found")
    lora.cleanup()
    quit(1)
else:
    print("RF95 LoRa mode ok")

# set frequency, power and mode
lora.set_frequency(433)
lora.set_tx_power(23)
#lora.set_modem_config(Bw31_25Cr48Sf512)
lora.set_modem_config(Bw125Cr45Sf128)
file=open("/home/pi/logfile.txt", "a")
file.write("\n")
file.write("C")
file.close()


#lora.send([0x00, 0x01, 0x02, 0x03])
#lora.wait_packet_sent()


header=[0xff, 0xff, 0, 0]
#lora.cleanup()
file=open("/home/pi/logfile.txt", "a")
file.write("\n")
file.write("After header")
file.close()



while True:
    print ("print now")
    lora.send(header+lora.str_to_data("$TELEMETRY TEST\0"))
    print ("just sent")
    lora.wait_packet_sent()

#    print ("print now")
#    lora.send(header+lora.str_to_data("$TELEMETRY TEST\0"))
   # lora.wait_packet_sent()

#    print ("TELEMETRY TEST")
    
#    data = lora.recv()
#    if(data):
#        print (data)
#        print ("Received: ")
#        for i in data:
#           print(chr(i), end="")
#        print()
#           #print ("NOw?: ")
#    else:
#        print("waiting")
    time.sleep(.1)
