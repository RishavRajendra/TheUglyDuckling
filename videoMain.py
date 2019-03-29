#!/usr/bin/env python
# -*- coding:utf-8 -*-
__author__ = "Rishav Rajendra, Benji Lee"
__license__ = "MIT"
__status__ = "Development"

import cv2
import numpy as np
import RPi.GPIO as GPIO
from picamera.array import PiRGBArray
from picamera import PiCamera
from constants import CAMERA_RESOLUTION, CAMERA_FRAMERATE, CENTER_DISTANCE_UP, CENTER_DISTANCE_DOWN, rotr, rotl, fwd, rev, buttonPin, ledPin
from get_stats_from_image import get_angle, get_distance, get_data, get_sensor_data, get_midpoint, mothership_angle
from nav.gridMovement import GridMovement
from nav.grid import Grid
import queue, threading, serial, time, math
from video_thread import VideoThread

import sys
sys.path.append("../tensorflow_duckling/models/research/object_detection/")
from image_processing import Model

import warnings
warnings.filterwarnings('ignore')

def wait_for_button(buttonPin, ledPin):
    print('Ready')
    GPIO.output(ledPin, GPIO.HIGH)
    #Prevent further code execution until button is pressed
    while GPIO.input(buttonPin) is not 0:
        pass
    GPIO.output(ledPin, GPIO.LOW)
    print('Executing')

def corrected_angle(angle, dist, cam_up=True):
    cd = CENTER_DISTANCE_UP if cam_up else CENTER_DISTANCE_DOWN
    sign = -1 if angle < 0 else 1
    angle = 180 - abs(angle)
    a = math.sqrt(math.pow(dist,2) + math.pow(cd, 2) - 2*dist*cd*math.cos(math.radians(angle)))
    angle_c = math.degrees(math.asin(math.sin(math.radians(angle))*dist/a))
    angle_b  = 180 -angle - angle_c
    print(angle_c, angle_b)
    if angle_c < angle_b:
        return math.floor(angle_c) * sign
    
    return math.floor(angle_b) * sign 

# TODO: Check if the block is actually picked up
def check_pick_up():
    pass
    
# TODO: Refine pick_up. Take care of edge cases.

# Change log
# [0.0.1] Benji
# - Changed sleep from 2 to 1.5; lowest fps is .75 so sleeping
# - for 1.5 seconds is the minimum delay that guarantees fresh video data
def pick_up(movement, pic_q):
    movement.cam_down()
    time.sleep(1.5)
    processed_frame, classes, boxes, scores = pic_q.get()
    object_stats = get_data(processed_frame, classes, boxes, scores)
    dist = 0
    angle = 0
    print("CAM DOWN: {}".format(object_stats))
    time.sleep(2)
    if not object_stats:
        print("CAM DOWN: NO OBJECT FOUND")
        #movement.cam_up()
        #movement.move(rev, 5)
        #time.sleep(2)
        approach(movement, pic_q, True, cam_up=False)
    else:
        print("CAM DOWN: Something located")
        for stats in object_stats:
            if(stats[0] > 0 and stats[0] < 7):
                print("MOVING TOWARDS TARGET")
                angle = corrected_angle(stats[1], stats[2])
                movement.turn(angle*-1)
                dist = math.ceil(stats[2]*.7)
                # move only 80% of the calculated distance to stop at pickup spot and not the front of the robot
                movement.move(fwd, dist)
            movement.pickup()
            movement.move(rev, dist)
            movement.turn(angle)


# TODO: Refine approach. Take care of edge cases.
# EDGE CASE 1: obstacle is in way of target
# Potential solution: go to another connected tile
# EDGE CASE 2: target not detected after two additional scans.

# Change log
# [0.0.1] Benji
# - Changed sleep from 2 to 1.5; lowest fps is .75 so sleeping
# - for 1.5 seconds is the minimum delay that guarantees fresh video data
def approach(movement, pic_q, first_call=True, cam_up=True):
    movement.cam_up() if cam_up else movement.cam_down()
    adj_degrees = 10 if first_call else -20
    angle = 0
    dist = 0
    time.sleep(1.5)
    processed_frame, classes, boxes, scores = pic_q.get()
    object_stats = get_data(processed_frame, classes, boxes, scores)
    print("CAM UP: {}".format(object_stats))
    target_found = False
    if object_stats:
        for stats in object_stats:
            if(stats[0] > 0 and stats[0] < 7):
                angle = corrected_angle(stats[1], stats[2]) 
                movement.turn(angle*-1)
                dist = stats[2] - 1
                movement.move(fwd, dist)
                target_found = True
                pick_up(movement, pic_q)
    if not target_found:
        movement.turn(adj_degrees)
        if first_call:
            approach(movement, pic_q, False, cam_up)
        if not first_call and cam_up:
            movement.turn(10)
            approach(movement, pic_q, True, False)
        movement.turn(adj_degrees*-1)
    else:
        movement.move(rev, dist)
        movement.turn(angle)

# Change log
# [0.0.1] Benji
# - Changed sleep from 2 to 1.5; lowest fps is .75 so sleeping
# - for 1.5 seconds is the minimum delay that guarantees fresh video data
def map(movement, pic_q):
    movement.cam_up()
    print(movement.facing)
    time.sleep(1.5)
    processed_frame, classes, boxes, scores = pic_q.get()
    object_stats = get_data(processed_frame, classes, boxes, scores)
    for stat in object_stats:
        obj_type = stat[0]
        angle = stat[1]
        dist = stat[2]
        print(obj_type, angle, dist)
        movement.map(obj_type, angle, dist)

def begin_round(movement, pic_q):
    for _ in range(8):
        movement.turn(45)
        map(movement, pic_q)

def follow_path(movement, pic_q):
    movement.find_path()
    while movement.path:
        print(movement.path)
        movement.follow_next_step()
        map(movement, pic_q)
        for obs in movement.get_obstacles():
            if obs in movement.path:
                movement.path.clear()
                movement.find_path()
    movement.face(movement.goal)

def get_sensor_data(serial):
    byteArr = b'\x08' + b'\x00' + b'\x00' + b'\x00'
    serial.write(byteArr)
    time.sleep(1)
    return int.from_bytes(serial.read(1),'little')

""" 
Approach mothership from close
Assuming that the robot is atleast pointed towards the mothership
Gets close the the mothership and calls mothership side angle
"""
def approach_mothership_side(movement, pic_q, serial):
    processed_frame, classes, boxes, scores = pic_q.get()
    object_stats = get_data(processed_frame, classes, boxes, scores)
    
    # Get current distance with IR sensor for validation
    distance_from_sensor = get_sensor_data(serial)
    print("Distance from sensor: {}".format(distance_from_sensor))
    print(object_stats)
    
    if object_stats:
        for stats in object_stats:
            if stats[0] == 8:
                movement.turn(corrected_angle(stats[1], stats[2]))
                if abs(distance_from_sensor - stats[2]) <= 5:
                    print("----------------Moving forward--------------")
                    movement.move(fwd, int((distance_from_sensor+stats[2])/2))
                    movement.cam_down()
                    mothership_side_angle(movement, pic_q, serial)
                else:
                    print("-------------Validation Failed-----------------")
                    # TODO: Handle edge cases
                    approach_mothership_side(movement, pic_q, serial)
    else:
        # TODO: Handle edge cases
        approach_mothership_side(movement, pic_q, serial)

"""
Turns the angle it believes the mothership is at from the front of the robot
"""
def mothership_side_angle(movement, pic_q, serial):
    distance_from_sensor = get_sensor_data(serial)
    # If sensor says the block is too far away, go forward 1 inch
    if distance_from_sensor != 0:
        movement.move(fwd, 1)
        mothership_side_angle(movement, pic_q, serial)

    processed_frame, classes, boxes, scores = pic_q.get()
    object_stats = get_midpoint(processed_frame, classes, boxes, scores)
    
    boxes_midpoint = []
    # Get midpoints of letters inside the mothership
    if object_stats:
        for stats in object_stats:
            if stats[0] > 0 and stats[0] < 7:
                boxes_midpoint.append(stats[1])
                    
    print(boxes_midpoint)
    if len(boxes_midpoint) >= 2:
        angle = mothership_angle(boxes_midpoint)
        # Turn to be somewhat parallel to the mothership
        movement.turn(angle)
        # TODO: strr or strl depending on the position to drop stuff
    else:
        # TODO: Have fun!
        pass


def main():
    # Initialize frame rate calculation
    frame_rate_calc = 1
    freq = cv2.getTickFrequency()
    font = cv2.FONT_HERSHEY_SIMPLEX

    objectifier = Model()

    # Start serial connection to arduino
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=2)
    time.sleep(1)

    # Initialize queues
    pic_q = queue.LifoQueue(5)
    command_q = queue.Queue()
    # Inizialize grid anf gridmovement
    grid = Grid(8,8)
    movement = GridMovement(grid, ser)
    # Initialize VideoThread
    vt = VideoThread(pic_q, objectifier)
    vt.start()
    
    # Setup GPIO
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(buttonPin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(ledPin, GPIO.OUT, initial=GPIO.LOW)

    wait_for_button(buttonPin, ledPin)
    time.sleep(2)
    

    drop_point = (7,7)
    begin_round(movement, pic_q)
    print(movement.get_obstacles())
    targs = [(1,6),(7,0)]
    
    # move to target, pick up
    # move to drop_point and drop block until no more targets
    for item in targs:
        movement.goal = item
        follow_path(movement, pic_q)
        approach(movement, pic_q)
        movement.goal = drop_point
        follow_path(movement, pic_q)
        movement.move(fwd, 6)
        movement.drop()
        movement.move(rev, 6)
    
                
    vt.join()
    #camera.close()

if __name__ == '__main__':
    main()
