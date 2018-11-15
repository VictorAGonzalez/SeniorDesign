#! /usr/bin/python
# Written by Dan Mandle http://dan.mandle.me September 2012
# License: GPL 2.0
 
import os
from gps import *
from time import *
import time
import threading
import math
import csv
import Adafruit_LSM303
import RPi.GPIO as GPIO
from time import sleep
import serial

GPIO.setmode(GPIO.BOARD)

lsm303 = Adafruit_LSM303.LSM303()

#Left Motor
Left1 = 16 # GPIO 23
Left2 = 18 # GPIO 24
MotorEn1 = 22 # GPIO 25

#Right Motor 
Right1 = 11 # GPIO 17
Right2 = 13 # GPIO 27
MotorEn2 = 15 # GPIO 22

GPIO.setup(Left1,GPIO.OUT)
GPIO.setup(Left2,GPIO.OUT)
GPIO.setup(MotorEn1,GPIO.OUT)
GPIO.setup(Right1,GPIO.OUT)
GPIO.setup(Right2,GPIO.OUT)
GPIO.setup(MotorEn2,GPIO.OUT)
 
gpsd = None #seting the global variable
 
os.system('clear') #clear the terminal (optional)

def getDegrees(lat1, long1, lat2, long2, head):
  dLat = math.radians(lat2-lat1)
  dLon = math.radians(long2 - long1)
  lat1 = math.radians(lat1)
  lat2 = math.radians(lat2)
  y = math.sin(dLon) * math.cos(lat2)
  x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dLon)
  brng = math.degrees(math.atan2(y, x))
  if (brng < 0):
      brng = 360 - math.fabs(brng)
  heading = brng - head
  if heading <= -180:
    heading = heading + 360
  return heading

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
 
if __name__ == '__main__':
  gpsp = GpsPoller() # create the thread

  with open('latlongtest.csv', 'r') as f:
    reader = csv.reader(f)
    lat_long = list(reader)

  lat_long = [[float(float(j)) for j in i] for i in lat_long]

  ser = serial.Serial(
    port='/dev/ttyACM0',
    baudrate = 9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
  )

  try:
    gpsp.start() # start it up
    for list in lat_long:
      input_lat = list[0]
      print input_lat
      input_long = list[1]
      print input_long
      latlong_bool = True

      while latlong_bool:
        accel, mag = lsm303.read()
        mag_x, mag_y, mag_z = mag
        az = (math.atan2(mag_z,mag_x)*(180/math.pi))
        if az < 0:
          az = 360 + az
        print 'Degree: ', az
        x = ser.readline()

        print "%2.8f,%2.8f" % (gpsd.fix.latitude, gpsd.fix.longitude) + x
        degree_to_turn = (getDegrees(gpsd.fix.latitude, gpsd.fix.longitude, input_lat, input_long, az))
        print(degree_to_turn)
        our_lat = round(gpsd.fix.latitude, 5)
        our_long = round(gpsd.fix.long, 5)
        in_lat = round(input_lat, 5)
        in_long = round(input_long, 5)
        if our_lat == in_lat and our_long == in_long:
          latlong_bool = False
          break
        elif degree_to_turn <= 15 and degree_to_turn >= -15 :
          print "FORWARD MOTION"
          GPIO.output(Left1,GPIO.HIGH)
          GPIO.output(Left2,GPIO.LOW)
          GPIO.output(MotorEn1,GPIO.HIGH)
          GPIO.output(Right4,GPIO.HIGH)
          GPIO.output(Right5,GPIO.LOW)
          GPIO.output(MotorEn2,GPIO.HIGH)
          sleep(3)
        elif degree_to_turn > 15:
          print "RIGHT TURN"
          GPIO.output(Left1,GPIO.HIGH)
          GPIO.output(Left2,GPIO.LOW)
          GPIO.output(MotorEn1,GPIO.HIGH)
          GPIO.output(Right4,GPIO.LOW)
          GPIO.output(Right5,GPIO.HIGH)
          GPIO.output(MotorEn2,GPIO.HIGH)
          sleep(3)
        elif degree_to_turn < -15 :
          print "LEFT TURN"
          GPIO.output(Left1,GPIO.LOW)
          GPIO.output(Left2,GPIO.HIGH)
          GPIO.output(MotorEn1,GPIO.HIGH)
          GPIO.output(Right4,GPIO.HIGH)
          GPIO.output(Right5,GPIO.LOW)
          GPIO.output(MotorEn2,GPIO.HIGH)
          sleep (3)
        else :
          print"STOP"
          GPIO.output(MotorEn1,GPIO.LOW)
          GPIO.output(MotorEn2,GPIO.LOW)
          sleep(3)
        time.sleep(0.5) #set to whatever
 
  except (KeyboardInterrupt, SystemExit): #when you press ctrl+c
    gpsp.running = False
    gpsp.join() # wait for the thread to finish what it's doing
