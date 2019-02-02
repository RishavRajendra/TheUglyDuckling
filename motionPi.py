# Functions the Raspberry Pi can call to communicate movement to the arduino

import serial, time

# Connect to arduino
ser = serial.Serial('/dev/ttyACM0', 9600, timeout=.1)
time.sleep(1)

# motion bytes
fwd = b'\x88'
rev = b'\x22'
rotl = b'\xAA'
rotr = b'\x00'
strl = b'\xA0'
strr = b'\x0A'

# Moves robot X distance (in centimenters) in direction given
def move(dir, dist):
	byteArr = dir+bytes([dist])
	ser.write(byteArr)
