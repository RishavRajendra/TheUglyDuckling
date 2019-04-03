#!/usr/bin/env python
# -*- coding:utf-8 -*-
__author__ = "Rishav Rajendra"
__license__ = "MIT"
__status__ = "Development"

import cv2
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
from constants import CAMERA_RESOLUTION, CAMERA_FRAMERATE, rotr, rotl
import get_stats_from_image
import queue, threading, serial, time

import sys
sys.path.append("../tensorflow_duckling/models/research/object_detection/")
from image_processing import Model

import warnings
warnings.filterwarnings('ignore')

def main():
    # Initialize frame rate calculation
    frame_rate_calc = 1
    freq = cv2.getTickFrequency()
    font = cv2.FONT_HERSHEY_SIMPLEX

    # Initialize camera and grab reference to the raw capture
    camera = PiCamera()
    camera.resolution = CAMERA_RESOLUTION
    camera.framerate = CAMERA_FRAMERATE
    rawCapture = PiRGBArray(camera, size=CAMERA_RESOLUTION)
    rawCapture.truncate(0)

    objectifier = Model()

    # Start serial connection to arduino
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=2)
    time.sleep(1)
    
    for frame1 in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        t1 = cv2.getTickCount()

        frame = np.copy(frame1.array)
        processed_frame, classes, boxes, scores = objectifier.predict(frame)

        cv2.putText(processed_frame, "FPS: {0:.2f}".format(frame_rate_calc),(30,50),font,1,(255,255,0),2,cv2.LINE_AA)

        result = (180,0)
        for i, b in enumerate(boxes[0]):
            inches = 0
            angle = 0
            # extract pixel coordinates of tdetected objects
            ymin = boxes[0][i][0]*300
            xmin = boxes[0][i][1]*300
            ymax = boxes[0][i][2]*300
            xmax = boxes[0][i][3]*300

            # Calculate mid_pount of the detected object
            mid_x = (xmax + xmin) / 2
            mid_y = (ymax + ymin) / 2

            height_of_object_pixels = ymax - ymin

            if classes[0][i] == 7:
                if scores[0][i] > 0.3:
                    inches = get_stats_from_image.get_distance(0, height_of_object_pixels)
            elif classes[0][i] == 1 or classes[0][i] == 2 or classes[0][i] == 3 or classes[0][i] == 4 or classes[0][i] == 5 or classes[0][i] == 6:
                if scores[0][i] > 0.3:
                    inches = get_stats_from_image.get_distance(1, height_of_object_pixels)
            elif classes[0][i] == 8 and scores[0][i] > 0.3:
                inches = get_stats_from_image.get_distance(5, height_of_object_pixels)
            elif classes[0][i] == 9 and scores[0][i] > 0.5:
                inches = get_stats_from_image.get_distance(3, height_of_object_pixels)
            elif classes[0][i] == 10 and scores[0][i] > 0.3:
                inches = get_stats_from_image.get_distance(4, height_of_object_pixels)
            if scores[0][i] > 0.3:
                angle = get_stats_from_image.get_angle(processed_frame, xmin, ymin, xmax, ymax)
                print('{} detected at {}{} {} inches away. Midpoint:({},{}). Height: {}'.format(classes[0][i],angle,chr(176),inches, int(mid_x), int(mid_y), int(height_of_object_pixels)))
        
        cv2.imshow('Object detector', processed_frame)

        t2 = cv2.getTickCount()
        time1 = (t2-t1)/freq
        frame_rate_calc=1/time1

        if cv2.waitKey(1) == ord('q'):
            break

        rawCapture.truncate(0)
    camera.close()

if __name__ == '__main__':
    main()
