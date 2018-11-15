#!/usr/bin/env python
import time
import serial

ser = serial.Serial(
        port='/dev/ttyACM0',
        baudrate = 9600,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
)

while 1:
        x=ser.readline()
	lat = 20.1
	long = 20.2 
        print "%.5f,%.5f" % (lat, long) + x
