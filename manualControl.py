#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = "Benji Lee, Rishav Rajendra"
__license__ = "MIT"
__status__ = "Development"

import serial
import keyboard
import time
import picamera
import queue
import threading
from subprocess import call
from motionPi import motionThread

# connect to arduino
ser = serial.Serial('/dev/ttyACM0', 9600, timeout=.1)
# might not need this
time.sleep(1)

# motion bytes
fwd = b'\x88'
rev = b'\x22'
rotl = b'\xAA'
rotr = b'\x00'
strl = b'\xA0'
strr = b'\x0A'

camera = picamera.PiCamera()
in_q = queue.Queue()
t1 = motionThread(in_q)
def pictures():
	a = 0
	camera.start_preview()
	while not t1.stoprequest.isSet():
		time.sleep(2)
		camera.capture("img{}.jpg".format(a))
		a = a + 1

# t2 = threading.Thread(target=pictures)			
t1.start()
# t2.start()
camera.start_preview()
time.sleep(2)
i = 0
while (i < 180/5):
	camera.capture("run35_img{}.jpg".format(i))
	in_q.put(['turn',(rotl, 5)])
	time.sleep(1)

t1.join()
# while not t1.stoprequest.isSet():
# 	if keyboard.is_pressed('w'):
# 		in_q.put(['move', (fwd, 5)])
# 	elif keyboard.is_pressed('s'):
# 		in_q.put(['move', (rev, 5)])
# 	elif keyboard.is_pressed('a'):
# 		in_q.put(['move', (strl, 5)])
# 	elif keyboard.is_pressed('d'):
# 		in_q.put(['move', (strr, 5)])
# 	elif keyboard.is_pressed('q'):
# 		in_q.put(['move', (rotl, 5)])
# 	elif keyboard.is_pressed('e'):
# 		in_q.put(['move', (rotr, 5)])
# 	elif keyboard.is_pressed('x'):
# 		t1.join()
# 		t2.join()
# 	time.sleep(.2)


