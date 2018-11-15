#! /usr/bin/python
# Written by Dan Mandle http://dan.mandle.me September 2012
# License: GPL 2.0
 
import os
from gps import *
from time import *
import time
import threading
import math
import Adafruit_LSM303

lsm303 = Adafruit_LSM303.LSM303()
 
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
  try:
    gpsp.start() # start it up
    while True:
      #It may take a second or two to get good data
      #print gpsd.fix.latitude,', ',gpsd.fix.longitude,'  Time: ',gpsd.utc
      accel, mag = lsm303.read()
      mag_x, mag_y, mag_z = mag
      az = (math.atan2(mag_z,mag_x)*(180/math.pi))
      if az < 0:
         az = 360 + az
      print 'Degree: ', az


      print "%2.8f,%2.8f,%s" % (gpsd.fix.latitude, gpsd.fix.longitude, gpsd.utc)
      print(getDegrees(25.78878, -80.39912, 25.78878, -80.39719, az))
      
 
      time.sleep(1) #set to whatever
 
  except (KeyboardInterrupt, SystemExit): #when you press ctrl+c
    gpsp.running = False
    gpsp.join() # wait for the thread to finish what it's doing
