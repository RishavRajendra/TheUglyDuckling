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
from constants import fwd, rev, buttonPin, ledPin
from get_stats_from_image import get_data, get_midpoint, mothership_angle, corrected_angle
from targetApproach import approach, check_pick_up
from nav.gridMovement import GridMovement
from misc import wait_for_button, get_sensor_data, align_corner, map, follow_path
from nav.grid import Grid
import queue, threading, serial, time, math
from video_thread import VideoThread

import sys
sys.path.append("../tensorflow_duckling/models/research/object_detection/")
from image_processing import Model

import warnings
warnings.filterwarnings('ignore')


   
        
    

def begin_round(movement, pic_q):
    for _ in range(8):
        movement.turn(45)
        map(movement, pic_q, True)
        


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
    processed_frame, classes, boxes, scores = pic_q.get()
    object_stats = get_data(processed_frame, classes, boxes, scores)
    
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

"""
Return true if obj is in our field of vision
else change goal and return False
"""
def verify_obj(movement, pic_q, obj):
    time.sleep(1.5)
    processed_frame, classes, boxes, scores = pic_q.get()
    object_stats = get_data(processed_frame, classes, boxes, scores)

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
    
    # Keep track of movements after approach is called
    approach_movement_list = queue.LifoQueue()

    wait_for_button(buttonPin, ledPin, GPIO)
    time.sleep(2)
    
    """
    if grid.sides:
        movement.goal = grid.sides[0]
    elif grid.slopes:
        movement.goal = (grid.slopes[0][0] -2, grid.slopes[0][1])
    follow_path(movement, pic_q)
    if verify_side(movement, pic_q):
        approach_mothership_side(movement, pic_q, ser)
    else:
        grid.slopes = [(grid.slopes[0][0] -1, grid.slopes[0][0])]
        movement.goal =(grid.slopes[0][0] +1, grid.slopes[0][0]-1)
        if verify_side(movement, pic_q):
            approach_mothership_side(movement, pic_q, ser)
    
    
    
    while True:
        pass
    """
   
    drop_point = (4, 7)
    #begin_round(movement, pic_q)
    #print(movement.get_obstacles())
    targs = [(4, 1),(3, 4)]
    
    # move to target, pick up
    # move to drop_point and drop block until no more targets
    for item in targs:
        movement.goal = item
        follow_path(movement, pic_q)
        approach(movement, pic_q, approach_movement_list)
        while not approach_movement_list.empty():
            move = approach_movement_list.get()
            if len(move) == 1:
                movement.turn(move[0])
            else:
                movement.move(move[0], move[1])
        movement.goal = drop_point
        follow_path(movement, pic_q)
        movement.move(fwd, 6)
        movement.drop()
        movement.move(rev, 6)
        movement.reset_servo()
    
                
    vt.join()
    #camera.close()

if __name__ == '__main__':
    main()
