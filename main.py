#!/usr/bin/env python
# -*- coding:utf-8 -*-
__author__ = "Rishav Rajendra"
__license__ = "MIT"
__status__ = "Development"

import cv2
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
from constants import CAMERA_RESOLUTION, CAMERA_FRAMERATE, fwd, rotr, rotl
import get_stats_from_image
import nav.gridMovement
import nav.grid
from nav.commandThread import CommandThread
import queue, threading, serial, time

import sys
sys.path.append("../tensorflow_duckling/models/research/object_detection/")
from image_processing import Model

import warnings
warnings.filterwarnings('ignore')

def take_picture(camera):
    camera.capture("command.jpg")
    return cv2.imread("command.jpg")
    
def get_data(processed_frame, classes, boxes, scores):
    result = []
    for i, b in enumerate(boxes[0]):
        if scores[0][i] > 0.5:
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
                inches = get_stats_from_image.get_distance(0, height_of_object_pixels)
            elif classes[0][i] == 2 or classes[0][i] == 3 or classes[0][i] == 4 or classes[0][i] == 5 or classes[0][i] == 6 or classes[0][i] == 7:
                inches = get_stats_from_image.get_distance(1, height_of_object_pixels)
                
            angle = get_stats_from_image.get_angle(processed_frame, xmin, ymin, xmax, ymax)
            #print('{} detected at {}{} {} inches away'.format(classes[0][i],angle,chr(176),inches))
            result.append([classes[0][i], angle, inches])
    return result

def main():
    # Initialize frame rate calculation
    frame_rate_calc = 1
    freq = cv2.getTickFrequency()
    font = cv2.FONT_HERSHEY_SIMPLEX
    
    #Initialise camera
    camera = PiCamera()
    camera.resolution = (300,300)

    objectifier = Model()

    # Start serial connection to arduino
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=2)
    time.sleep(1)

    # Initialize queue
    in_q = queue.Queue()
    
    # Inizialize grid anf gridmovement
    grid = Grid(8,8)
    movement = GridMovement(grid, in_q)

    # Initialize commandThread
    lock = threading.Lock()
    ct = CommandThread(in_q, ser, lock)
    ct.start()
    
    #Example usage
    for _ in range(4):
        frame = take_picture(camera)
        processed_frame, classes, boxes, scores = objectifier.predict(frame)
        object_stats = get_data(processed_frame, classes, boxes, scores)
        print(object_stats)
        for stat in object_stats:
            if stat[0] > 1 and stat[0] < 9:
                movement.map(stat)

        in_q.push(['turn', (rotl, 90)])
        movement.facing = movement.facing - 90
        movement.trim_facing()

    movement.find_path()
    movement.follow_path()        
    
    cv2.waitKey(0)

        
    #camera.close()

if __name__ == '__main__':
    main()
