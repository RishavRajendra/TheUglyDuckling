#!/usr/bin/env python
# -*- coding:utf-8 -*-
__author__ = "Rishav Rajendra"
__license__ = "MIT"
__status__ = "Development"

import cv2
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
from constants import CAMERA_RESOLUTION, CAMERA_FRAMERATE, CENTER_DISTANCE, rotr, rotl, fwd
from get_stats_from_image import get_angle, get_distance
from nav.gridMovement import GridMovement
from nav.grid import Grid
from nav.command import Command
import queue, threading, serial, time, math
from video_thread import VideoThread

import sys
sys.path.append("../tensorflow_duckling/models/research/object_detection/")
from image_processing import Model

import warnings
warnings.filterwarnings('ignore')

def get_data(processed_frame, classes, boxes, scores):
    result = []
    for i, b in enumerate(boxes[0]):
        if scores[0][i] > 0.35:
            inches = 0
            #extract pixel coordinates of detected objects
            ymin = boxes[0][i][0]*300
            xmin = boxes[0][i][1]*300
            ymax = boxes[0][i][2]*300
            xmax = boxes[0][i][3]*300

            # Calculate mid_pount of the detected object
            mid_x = (xmax + xmin) / 2
            mid_y = (ymax + ymin) / 2

            height_of_object_pixels = ymax - ymin

            if classes[0][i] == 8:
                inches = get_distance(0, height_of_object_pixels)
            elif classes[0][i] == 2 or classes[0][i] == 3 or classes[0][i] == 4 or classes[0][i] == 5 or classes[0][i] == 6 or classes[0][i] == 7:
                inches = get_distance(1, height_of_object_pixels)
                
            angle = get_angle(processed_frame, xmin, ymin, xmax, ymax)
            #print('{} detected at {}{} {} inches away'.format(classes[0][i],angle,chr(176),inches))
            result.append([classes[0][i], angle, inches])
    return result

def corrected_angle(angle, dist):
    angle = 180 - angle
    a = math.sqrt(math.pow(dist,2) + math.pow(CENTER_DISTANCE, 2) - 2*dist*CENTER_DISTANCE*math.cos(math.radians(angle)))
    angle_c = math.asin(math.sin(math.radians(angle))*dist/a)
    return math.ceil(180-angle-math.degrees(angle_c))

def approach(command_q, pic_q, commands, first_call=True):
    adj_degrees = 10 if first_call else 20
    adj_dir = rotr if first_call else rotl
    processed_frame, classes, boxes, scores = pic_q.get()
    object_stats = get_data(processed_frame, classes, boxes, scores)
    print(object_stats)
    if not object_stats:
        command_q.put(['turn',(adj_dir, adj_degrees, False)])
        commands.execute()
        time.sleep(2)
        approach(command_q, pic_q, commands, False)
    else:
        target_found = False
        for stats in object_stats:
            angle = stats[1]
            dist = stats[2]
            obj_type = stats[0]
            #print(obj_type)
            #print(angle)
            #print(dist) 
            if(obj_type > 1 and obj_type < 8):
                turn_dir = rotl if angle < 0 else rotr
                angle = corrected_angle(abs(angle), dist) 
                command_q.put(['turn', (turn_dir, angle, False)])
                command_q.put(['move',(fwd, dist )])
                commands.execute()
                target_found = True
                break
        if not target_found:
            command_q.put(['turn',(adj_dir, adj_degrees, False)])
            commands.execute()
            time.sleep(2)
            approach(command_q, pic_q, commands, False)
    
    

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
    grid = Grid(8,8)
    movement = GridMovement(grid, command_q, commands)
    # Initialize VideoThread
    vt = VideoThread(pic_q, objectifier)
    vt.start()

    #Testing approach
    approach(command_q, pic_q, commands) 
                
    vt.join()
    #camera.close()

if __name__ == '__main__':
    main()
