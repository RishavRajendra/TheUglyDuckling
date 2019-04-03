#!/usr/bin/env python
# -*- coding:utf-8 -*-
__author__ = "Rishav Rajendra, Benji Lee"
__license__ = "MIT"
__status__ = "Development"

from get_stats_from_image import get_data
import time

def wait_for_button(buttonPin, ledPin, GPIO, log):
    log.info('Ready')
    GPIO.output(ledPin, GPIO.HIGH)
    #Prevent further code execution until button is pressed
    while GPIO.input(buttonPin) is not 0:
        pass
    GPIO.output(ledPin, GPIO.LOW)
    log.info('Executing') 

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

"""
Change log
    -[0.0.1] Benji
        --- Changed sleep from 2 to 1.5; lowest fps is .75 so sleeping
        --- for 1.5 seconds is the minimum delay that guarantees fresh video data
"""
def map_movement(movement, pic_q, log, beginning=False):
    movement.cam_up()
    log.info(movement.facing)
    time.sleep(3)
    object_stats = get_data(pic_q)
    for stat in object_stats:
        obj_type = stat[0]
        angle = stat[1]
        dist = stat[2]
        log.debug('Map Movement:- Object type: {}, Angle: {}, Dist: {}'.format(obj_type, angle, dist))
        if obj_type == 9:
            dist = dist + 3
        if beginning:
            movement.map(obj_type, angle, dist)
        elif obj_type == 7:
            movement.map(obj_type, angle, dist)
            
"""
Change Log
    [0.0.1]
        --- Added parameter to allow including goal in path
"""
def follow_path(movement, pic_q, log, include_goal=False):
    movement.find_path(include_goal)
    while movement.path:
        log.debug('Movement path: {}'.format(movement.path))
        movement.follow_next_step()
        map_movement(movement, pic_q)
        for obs in movement.get_obstacles():
            if obs in movement.path:
                movement.path.clear()
                movement.find_path(include_goal)
    if not movement.goal == movement.current:
        movement.face(movement.goal)

# Called at the start of the round. 
# Maps relative locations to obstacles and mothership from start point
def begin_round(movement, pic_q):
    for _ in range(8):
        movement.turn(45)
        map_movement(movement, pic_q, True)

"""
Go to Home square
"""
def go_home(movement, pic_q):
    if not movement.goal == movement.current:
        movement.path.clear()
        movement.goal = (4,4)
        follow_path(movement, pic_q, True)