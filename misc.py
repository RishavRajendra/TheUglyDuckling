#!/usr/bin/env python
# -*- coding:utf-8 -*-
__author__ = "Rishav Rajendra, Benji Lee"
__license__ = "MIT"
__status__ = "Development"

from get_stats_from_image import get_data
import time

def wait_for_button(buttonPin, ledPin, GPIO):
    print('Ready')
    GPIO.output(ledPin, GPIO.HIGH)
    #Prevent further code execution until button is pressed
    while GPIO.input(buttonPin) is not 0:
        pass
    GPIO.output(ledPin, GPIO.LOW)
    print('Executing') 

"""
Returns the average distance of the object infront of the mothership
Mid-range IR sensor connected to the Arduino Mega 2560
Raspberry pi gets data using serial
"""
def get_sensor_data(serial):
    byteArr = b'\x08' + b'\x00' + b'\x00' + b'\x00'
    serial.write(byteArr)
    time.sleep(1)
    left = int.from_bytes(serial.read(1),'little')
    right = int.from_bytes(serial.read(1),'little')
    return left, right
    
# TODO
def align_corner(movement, pic_q):
	pass
