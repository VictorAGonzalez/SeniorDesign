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
  return (brng - head)

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

  try:
    gpsp.start() # start it up
    for list in lat_long:
      for number in list:
        input_lat = number
        print input_lat
        next(list, None)
        input_long = number
        print input_long
        latlong_bool = True
        while latlong_bool:



          accel, mag = lsm303.read()
          mag_x, mag_y, mag_z = mag
          az = (math.atan2(mag_z,mag_x)*(180/math.pi))
          if az < 0:
            az = 360 + az
          print 'Degree: ', az


          print "%2.8f,%2.8f" % (gpsd.fix.latitude, gpsd.fix.longitude)
          degree_to_turn = (getDegrees(gpsd.fix.latitude, gpsd.fix.longitude, input_lat, input_long, az))
          print(degree_to_turn)
          if gpsd.fix.latitude == input_lat and gpsd.fix.long == input_longitude:
            latlong_bool = False

          time.sleep(0.5) #set to whatever
 
  except (KeyboardInterrupt, SystemExit): #when you press ctrl+c
    gpsp.running = False
    gpsp.join() # wait for the thread to finish what it's doing
