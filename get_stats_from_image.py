#!/usr/bin/env python
# -*- coding:utf-8 -*-
__author__ = "Rishav Rajendra"
__license__ = "MIT"
__status__ = "Development"

import cv2
import math
from constants import CENTER_LINEx1y1, CENTER_LINEx2y2, PICKUP_SWEET_SPOTx1y1, PICKUP_SWEET_SPOTx2y2, PIXEL_PER_MM, ERROR_VAL, FOCAL_LENGTH, OBSTACLE_HEIGHT, TARGET_HEIGHT

# angle = arctan((m2-m1)/(1+(m1*m2)))
def get_angle(image, xmin, ymin, xmax, ymax):
    center = (int((xmin + xmax)/2), int((ymin + ymax)/2))
    cv2.line(image, center, CENTER_LINEx2y2, (0,0,0))
    m1 = ((CENTER_LINEx1y1[1] - CENTER_LINEx2y2[1])+0.001)/((CENTER_LINEx1y1[0] - CENTER_LINEx2y2[0])+0.001)
    m2 = ((center[1] - CENTER_LINEx2y2[1])+0.001)/((center[0] - CENTER_LINEx2y2[0])+0.001)
    div = (m2-m1) / (1+(m1*m2))
    return int(math.degrees(math.atan(div)))

def pick_up_get_angle(image, xmin, ymin, xmax, ymax):
    center = (int((xmin + xmax)/2), int((ymin + ymax)/2))
    cv2.line(image, center, PICKUP_SWEET_SPOTx2y2, (0,0,0))
    m1 = ((PICKUP_SWEET_SPOTx1y1[1] - PICKUP_SWEET_SPOTx2y2[1])+0.001)/((PICKUP_SWEET_SPOTx1y1[0] - PICKUP_SWEET_SPOTx2y2[0])+0.001)
    m2 = ((center[1] - PICKUP_SWEET_SPOTx2y2[1])+0.001)/((center[0] - PICKUP_SWEET_SPOTx2y2[0])+0.001)
    div = (m2-m1) / (1+(m1*m2))
    return int(math.degrees(math.atan(div)))

# object_type = 0 (obstacle)
# object type = 1 (target)
def get_distance(object_type, height_of_object_pixels):
    if object_type == 0:
        return int(((OBSTACLE_HEIGHT*FOCAL_LENGTH)/((height_of_object_pixels-ERROR_VAL)/PIXEL_PER_MM))/10)
    if object_type == 1:
        return int(((TARGET_HEIGHT*FOCAL_LENGTH)/((height_of_object_pixels-ERROR_VAL)/PIXEL_PER_MM))/10)
        
# object_type = 0 (obstacle)
# object type = 1 (target)
def cam_down_distance(height_of_object_pixels):
    return int(((CAM_DOWN_TARGET_HEIGHT*FOCAL_LENGTH)/((height_of_object_pixels-ERROR_VAL)/PIXEL_PER_MM))/10)

# Returns [object_name, angle, inches]
def get_data(processed_frame, classes, boxes, scores):
    result = []
    for i, b in enumerate(boxes[0]):
        if scores[0][i] > 0.5:
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
            #print('{} detected at {}{} {} inches away. Midpoint:({},{}). Height: {}'.format(classes[0][i],angle,chr(176),inches, int(mid_x), int(mid_y), int(height_of_object_pixels)))
            result.append([classes[0][i], angle, inches])
    return result

