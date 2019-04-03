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
from get_stats_from_image import get_angle, get_distance, get_data, get_midpoint, mothership_angle
from misc import get_sensor_data
from nav.gridMovement import GridMovement
from nav.grid import Grid
import queue, threading, serial, time, math
from video_thread import VideoThread

import sys
sys.path.append("../../tensorflow_duckling/models/research/object_detection/")
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

"""
Change log
    -[0.0.1] Benji
        --- Changed sleep from 2 to 1.5; lowest fps is .75 so sleeping
        --- for 1.5 seconds is the minimum delay that guarantees fresh video data
"""
def pick_up(movement, pic_q,approach_movement_list):
    movement.cam_down()
    time.sleep(1.5)
    processed_frame, classes, boxes, scores = pic_q.get()
    object_stats = get_data(processed_frame, classes, boxes, scores)
    dist = 0
    angle = 0
    print("CAM DOWN: {}".format(object_stats))
    if not object_stats:
        print("CAM DOWN: NO OBJECT FOUND")
        #movement.cam_up()
        #movement.move(rev, 5)
        #time.sleep(2)
        approach(movement, pic_q,approach_movement_list, True, cam_up=False)
    else:
        print("CAM DOWN: Something located")
        for stats in object_stats:
            if(stats[0] > 0 and stats[0] < 7):
                print("MOVING TOWARDS TARGET")
                angle = corrected_angle(stats[1], stats[2])
                movement.turn(angle*-1)
                approach_movement_list.put([angle])
                dist = math.ceil(stats[2]*.7)
                # move only 70% of the calculated distance to stop at pickup spot and not the front of the robot
                movement.move(fwd, dist)
                approach_movement_list.put([rev, dist])
                movement.pickup()
            


# TODO: Refine approach. Take care of edge cases.
# EDGE CASE 1: obstacle is in way of target
# Potential solution: go to another connected tile
# EDGE CASE 2: target not detected after two additional scans.

"""
Change log
    -[0.0.1] Benji
        --- Changed sleep from 2 to 1.5; lowest fps is .75 so sleeping
        --- for 1.5 seconds is the minimum delay that guarantees fresh video data
"""
def approach(movement, pic_q, approach_movement_list, first_call=True, cam_up=True):
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
                approach_movement_list.put([angle])
                dist = stats[2] - 1
                movement.move(fwd, dist)
                approach_movement_list.put([rev, dist])
                target_found = True
                pick_up(movement, pic_q,approach_movement_list)
    if not target_found:
        movement.turn(adj_degrees)
        approach_movement_list.put([-adj_degrees])
        if first_call:
            approach(movement, pic_q,approach_movement_list, False, cam_up)
        if not first_call and cam_up:
            movement.turn(10)
            approach_movement_list.put([-10])
            approach(movement, pic_q,approach_movement_list, True, False)
        movement.turn(adj_degrees*-1)
        approach_movement_list.put([adj_degrees])

"""
Change log
    -[0.0.1] Benji
        --- Changed sleep from 2 to 1.5; lowest fps is .75 so sleeping
        --- for 1.5 seconds is the minimum delay that guarantees fresh video data
"""
def map(movement, pic_q, beginning=False):
    movement.cam_up()
    print(movement.facing)
    time.sleep(3)
    object_stats = get_data(pic_q)
    for stat in object_stats:
        obj_type = stat[0]
        angle = stat[1]
        dist = stat[2]
        print(obj_type, angle, dist)
        if obj_type == 9:
            dist = dist + 3
        if beginning:
            movement.map(obj_type, angle, dist)
        elif obj_type == 7:
            movement.map(obj_type, angle, dist)

def begin_round(movement, pic_q):
    for _ in range(8):
        movement.turn(45)
        map(movement, pic_q, True)
        
"""
Change Log
    [0.0.1]
        --- Added parameter to allow including goal in path
"""
def follow_path(movement, pic_q, include_goal=False):
    movement.find_path(include_goal)
    while movement.path:
        print(movement.path)
        movement.follow_next_step()
        map(movement, pic_q)
        for obs in movement.get_obstacles():
            if obs in movement.path:
                movement.path.clear()
                movement.find_path(include_goal)
    if not movement.goal == movement.current:
        movement.face(movement.goal)

    
"""
Go to Home square
"""
def go_home(movement, pic_q):
    if not movement.goal == movement.current:
        movement.path.clear()
        movement.goal = (4,4)
        follow_path(movement, pic_q, True)

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
    
    # Keep track of movements after approach is called
    approach_movement_list = queue.LifoQueue()

    wait_for_button(buttonPin, ledPin)
    time.sleep(2)
    
    
    begin_round(movement, pic_q)
    map_mothership(movement, pic_q)
    print("Mothership is located in the following tiles: ", grid.mothership)
    if grid.mothership:
        approach_mothership_side(movement, pic_q, ser)
                
    vt.join()
    #camera.close()

if __name__ == '__main__':
    main()
