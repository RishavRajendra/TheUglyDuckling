#!/usr/bin/env python
# -*- coding:utf-8 -*-
__author__ = "Rishav Rajendra"
__license__ = "MIT"
__status__ = "Development"

import cv2
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
from constants import CAMERA_RESOLUTION, CAMERA_FRAMERATE, rotr, rotl, fwd
from get_stats_from_image import get_angle, get_distance, get_data
import nav.gridMovement
import nav.grid
from nav.command import Command
import queue, threading, serial, time, math
from video_thread import VideoThread

import sys
sys.path.append("../../tensorflow_duckling/models/research/object_detection/")
from image_processing import Model

import warnings
warnings.filterwarnings('ignore')

def corrected_angle(angle, dist):
    angle = 180 - angle
    a = math.sqrt(math.pow(dist,2) + math.pow(3.5, 2) - 2*dist*3.5*math.cos(math.radians(angle)))
    angle_c = math.asin(math.sin(math.radians(angle))*dist/a)
    return math.ceil(180-angle-math.degrees(angle_c))

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
    #Inizialize commands
    commands = Command(command_q, ser)
    # Inizialize grid anf gridmovement
    grid = nav.grid.Grid(8,8)
    movement = nav.gridMovement.GridMovement(grid, command_q)
    # Initialize VideoThread
    vt = VideoThread(pic_q, objectifier)
    vt.start()
    
    # Pickup routine beta
    processed_frame, classes, boxes, scores = pic_q.get()
    object_stats = get_data(processed_frame, classes, boxes, scores)
    #for stat in object_stats:
        #if stat[0] > 1 and stat[0] < 8:
            #print("{} detected at {} angle {} inches away".format(stat[0], stat[1], stat[2]))
            #turn_dir = rotl if angle < 0 else rotr
            #angle = corrected_angle(abs(stat[0]), stat[2]) 
            #command_q.put(['turn', (turn_dir, angle, False)])
            #commands.execute()
    angle = object_stats[0][1]
    dist = object_stats[0][2]
    turn_dir = rotl if angle < 0 else rotr
    angle = corrected_angle(abs(angle), dist) 
    command_q.put(['turn', (turn_dir, angle, False)])
    commands.execute()
    
    #while(dist > 9):
        #command_q.put(['move',(fwd, math.floor(dist/2))])
        #commands.execute()
        #time.sleep(2) 
        #processed_frame, classes, boxes, scores = pic_q.get()
        #object_stats = get_data(processed_frame, classes, boxes, scores)
        #angle = object_stats[0][1]
        #dist = object_stats[0][2]
        #turn_dir = rotl if angle < 0 else rotr
        #angle = corrected_angle(abs(angle), dist) 
        #command_q.put(['turn', (turn_dir, angle, False)])
        #commands.execute()
	
    #command_q.put(['move',(fwd, math.floor(dist/2))])
    #commands.execute() 
    
    # Pickup block
    #processed_frame, classes, boxes, scores = pic_q.get()
    #object_stats = get_data(processed_frame, classes, boxes, scores)
    #for stats in object_stats:
        #if stats[0] > 1 and stats[0] < 8:
            #print("{} detected at {} angles {} inches away.".format(stats[0], stats[1], stats[2]))
            #turn_dir = rotl if angle < 0 else rotr
            #turn_angle = corrected_angle(abs(stats[1]), stats[2])
            #command_q.put(['turn', (turn_dir, turn_angle, False)])
            #commands.execute()
            #break
                
    #print(grid.obstacles)
    #print(grid.targets)
    #movement.goal = (4,7)
    #movement.find_path()
    #print(movement.facing)
    #print(movement.path)
    #movement.follow_path()
    #commands.execute()
    vt.join()
    #camera.close()

if __name__ == '__main__':
    main()

