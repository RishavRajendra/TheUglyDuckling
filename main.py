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
from mothership_commands import map_mothership, approach_mothership_side
from nav.gridMovement import GridMovement
from misc import wait_for_button, follow_path, begin_round
from nav.grid import Grid
import queue, threading, serial, time, math
from video_thread import VideoThread

import sys
sys.path.append("../tensorflow_duckling/models/research/object_detection/")
from image_processing import Model

import warnings
warnings.filterwarnings('ignore')
    
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

    wait_for_button(buttonPin, ledPin, GPIO)
    time.sleep(2)
    
    begin_round(movement, pic_q)
    map_mothership(movement, pic_q)
    print("Mothership is located in the following tiles: ", grid.mothership)
    mothership_angle, dist, side_angle = approach_mothership_side(movement, pic_q, ser)
    print(mothership_angle, side_angle)
    go_home(movement, pic_q)
    
    # This should be the coordinate of the side
    # Could not figure out how to get that.
    # Need sleep
    drop_point = (4,2)
    
    targs = [(4,7)]
    for items in targs:
        movement.goal = items
        follow_path(movement, pic_q)
        approach(movement, pic_q)
        go_home(movement, pic_q)
        movement.goal = drop_point
        follow_path(movement, pic_q)
        movement.turn(-1*mothership_angle)
        movement.move(fwd, dist)
        movement.turn(-1*side_angle)
        movement.drop()
    
    vt.join()
    #camera.close()

if __name__ == '__main__':
    main()
