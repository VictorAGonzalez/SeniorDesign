import serial
import time


message = 5

arduino = serial.Serial('/dev/ACM0',baudrate=9600,timeout=3.0)
time.sleep(2)
arduino.write(bytes([message]))
a=arduino.read(1)
print(a)
arduino.close()
