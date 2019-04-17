import serial
import time
import struct
import getch

from ctypes import *

ser = serial.Serial(port='/dev/ttyS0',baudrate=9600)
print("Connected to: " + ser.portstr)

class Message(Structure):
    _fields_ = [("velocity", c_float),
                ("theta", c_float),
                ("mode", c_uint32)]

#struct
	#velocity
	#theta
	#mode
	
while true:
	inAngle = 0
	inVelocity = 0
	inMode = 1
	
	print("angle: ")
	while inAngle = 0:
        inAngle = getch.getche()
		
	print("velocity: ")
    while inVelocity = 0:
        inVelocity = getch.getche()
