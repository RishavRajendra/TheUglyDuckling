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
from get_stats_from_image import get_angle, get_distance, get_data, get_sensor_data, get_midpoint, mothership_angle, corrected_angle, get_closest_target
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

# TODO: Check if the block is actually picked up
def check_pick_up():
    pass

def move_to_target(movement, angle, distance):
    movement.turn(angle)
    movement.move(fwd, distance)

def move_back_from_target(movement, angle, distance):
    movement.move(rev, distance)
    movement.turn(angle)
    
# TODO: Refine pick_up. Take care of edge cases.

"""
Change log
    -[0.0.1] Benji
        --- Changed sleep from 2 to 1.5; lowest fps is .75 so sleeping
        --- for 1.5 seconds is the minimum delay that guarantees fresh video data
    -[0.0.1] Risahv
        --- Remove recursion. Not getting satisfying results.
"""
def pick_up(movement, pic_q, serial):
    movement.cam_down()
    # Let the imaging catch_up
    time.sleep(3)
    # Get data from the closest object
    target_id, angle, inches, midpoint = get_closest_target(pic_q, True)
    if target_id == 0:
        print("Nothing found in pick_up")
    else:
        if midpoint[0] > 125 and midpoint[0] < 230 and midpoint[1] > 255:
            movement.pickup()
        else:
            correctedAngle = corrected_angle(angle, inches, False)
            movement.turn(-correctedAngle)

            # Get another picture as distance can get wacky at an angle
            target_id, angle, inches, midpoint = get_closest_target(pic_q, True)
            move_to_target(movement,-1*corrected_angle(angle, inches, False), math.ceil(inches*0.7))
            movement.pickup()
            move_back_from_target(movement,corrected_angle(angle, inches), math.ceil(inches*0.7))
            # Reverse movements
            movement.turn(correctedAngle)

    movement.cam_up()

# TODO: Refine approach. Take care of edge cases.
# EDGE CASE 1: obstacle is in way of target
# Potential solution: go to another connected tile
# EDGE CASE 2: target not detected after two additional scans.
# EDGE CASE 3: second object detected in background

"""
Change log
    -[0.0.1] Benji
        --- Changed sleep from 2 to 1.5; lowest fps is .75 so sleeping
        --- for 1.5 seconds is the minimum delay that guarantees fresh video data
    -[0.0.2] Rishav
        --- Remove recursion as results not satisfying.
"""
def approach(movement, pic_q, serial):
    movement.cam_up()
    # Get data of the closest object
    target_id, angle, inches = get_closest_target(pic_q)
    if target_id == 0:
        print("Nothing found in approach")
    else:
        print("Approach CAM_UP: [{}, {}, {}]".format(target_id, angle, inches))
        # move towards the target
        move_to_target(movement,-1*corrected_angle(angle, inches), inches)
        #Realign with block
        pick_up(movement, pic_q, serial)
        # Reverse movement from the target
        move_back_from_target(movement, corrected_angle(angle, inches), inches)

"""
Change log
    -[0.0.1] Benji
        --- Changed sleep from 2 to 1.5; lowest fps is .75 so sleeping
        --- for 1.5 seconds is the minimum delay that guarantees fresh video data
"""
def map(movement, pic_q, beginning=False):
    movement.cam_up()
    print(movement.facing)
    time.sleep(1.5)
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

def get_sensor_data(serial):
    byteArr = b'\x08' + b'\x00' + b'\x00' + b'\x00'
    serial.write(byteArr)
    time.sleep(1)
    return int.from_bytes(serial.read(1),'little')
    
"""
Go to Home square
"""
def go_home(movement, pic_q):
    if not movement.goal == movement.current:
        movement.path.clear()
        movement.goal = (4,4)
        follow_path(movement, pic_q, True)

#####
##### Mothership code
####

""" 
Approach mothership from close
Assuming that the robot is atleast pointed towards the mothership
Gets close the the mothership and calls mothership side angle
"""
def approach_mothership_side(movement, pic_q, serial):
    time.sleep(1.5)
    object_stats = get_data(pic_q)
    
    # Get current distance with IR sensor for validation
    distance_from_sensor = get_sensor_data(serial)
    print("Distance from sensor: {}".format(distance_from_sensor))
    print(object_stats)
    
    if object_stats:
        for stats in object_stats:
            if stats[0] == 8:
                movement.turn(-corrected_angle(stats[1], stats[2]))
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

    object_stats = get_midpoint(pic_q)
    
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

"""
Return true if obj is in our field of vision
else change goal and return False
"""
def verify_obj(movement, pic_q, obj):
    time.sleep(1.5)
    object_stats = get_data(pic_q)

    for stats in object_stats:
        if stats[0] == obj:
            return True
    
    facing = movement.facing
    tx, ty = movement.goal[0], movement.goal[1]
    
    if obj == 9:
        c = 1 if tx > 4 else -1
        d = 1 if ty > 4 else -1
        if (facing == 90 or facing == 270):
            movement.goal = (tx + c, ty)
        else:
            movement.goal = (tx, ty + d) 

    return False

def locate_obj(movement, pic_q, obj):
    facing = movement.facing
    cx, cy = movement.current[0], movement.current[1]
    tx, ty = movement.goal[0], movement.goal[1]

    if facing == 90:
        angle = 45 if tx < cx else -45
    elif facing == 270:
        angle = -45 if tx < cx else 45
    elif facing == 0:
        angle = -45 if ty < cy else 45
    else:
        angle = 45 if ty < cy else -45

    for _ in range(2):
        movement.turn(angle)
        if verify_obj(movement, pic_q, obj):
            return True 

    return False
"""
Mapping mothership by side because we detected a side
"""
def map_by_side(movement, pic_q, serial, goal=None):
    print("-------Checking side-------")
    # Move to initial mapped side
    movement.goal = movement.grid.sides[0] if goal == None else goal 
    follow_path(movement, pic_q)
    
    # Verify it's location
    # If it's where we think it is - awesome-

    if verify_obj(movement,pic_q, 8):
        movement.grid.sides.clear()
        map(movement, pic_q, True)
        # current location is access_point. Map it
        # approach
        approach_mothership_side(movement, pic_q, serial)
    # else if we locate it
    elif locate_obj(movement, pic_q, 8):
        # current location is access_point. Map it
        # we should be facing the side so approach 
        approach_mothership_side(movement, pic_q, serial)
    # else go home - you're drunk - try again later
    else:
        print("Go Home")
        go_home(movement, pic_q)

"""
Mapping mothership by slope because we didn't find a side
"""
def map_by_slope(movement, pic_q, serial, first_call=True):
    print("-------Checking slope-------")
    # Move to initial mapped slope
    movement.goal = movement.grid.slopes[0] if first_call else movement.goal
    follow_path(movement, pic_q)

    # Verify it's location
    # If it's where we think it is
    if verify_obj(movement, pic_q, 9):
        movement.grid.slopes.clear()
        map(movement, pic_q, True)
        slope = movement.grid.slopes[0]
        # we try to map by side for both possibilities
        tx, ty = slope[0], slope[1]

        c = 1 if tx > 4 else -1
        d = 1 if ty > 4 else -1

        movement.grid.sides = [(tx + c, ty),(tx, ty + d)]
        movement.grid.access_points = [(tx + c, ty - d),(tx - c - c, ty + d)]
        print("access points are ", movement.grid.access_points)

        for i in range(2):
            movement.goal = movement.grid.access_points[i]
            follow_path(movement, pic_q, True)
            map_by_side(movement, pic_q, serial, movement.grid.sides[i])
            # we'll know the first try was successful if grid.mothership has objs in it 
            
    # Else we try one more time with new guess
    elif not first_call: 
        map_by_slope(movement, pic_q, serial, False)
    # Else go home - you're drunk - try again later
    else:
        print("Go Home")
        go_home(movement, pic_q)


def map_mothership(movement, pic_q, serial):
    if movement.grid.sides:
        map_by_side(movement, pic_q, serial)
    else:
        map_by_slope(movement, pic_q, serial)

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
    
    print(movement.facing)
    approach(movement, pic_q, ser)
    print(movement.facing)
                
    vt.join()
    #camera.close()

if __name__ == '__main__':
    main()
