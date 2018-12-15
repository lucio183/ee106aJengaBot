#!/usr/bin/python
import serial
import syslog
import time

#The following line is for serial over GPIO
port = '/dev/ttyACM1' # note I'm using Mac OS-X


ard = serial.Serial(port,9600,timeout=5)
#time.sleep(2) # wait for Arduino

while (True):
    # Serial write section


    ard.flush()

    time.sleep(.15) # I shortened this to match the new value in your Arduino code

    # Serial read section
    msg = ard.read(ard.inWaiting()) # read all characters in buffer
    number = msg.split(",")
    #print(len(number))
    if (len(number) > 2):
        print(number[-2])

else:
    print "Exiting"
exit()