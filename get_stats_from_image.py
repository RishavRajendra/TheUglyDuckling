#!/usr/bin/env python
# -*- coding:utf-8 -*-
__author__ = "Rishav Rajendra"
__license__ = "MIT"
__status__ = "Development"

import cv2
import math
from constants import CENTER_LINEx1y1, CENTER_LINEx2y2, PIXEL_PER_MM, ERROR_VAL, FOCAL_LENGTH, OBSTACLE_HEIGHT, TARGET_HEIGHT

def get_angle(image, xmin, ymin, xmax, ymax):
    center = (int((xmin + xmax)/2), int((ymin + ymax)/2))
    cv2.line(image, center, CENTER_LINEx2y2, (0,0,0))
    m1 = ((CENTER_LINEx1y1[1] - CENTER_LINEx2y2[1])+0.001)/((CENTER_LINEx1y1[0] - CENTER_LINEx2y2[0])+0.001)
    m2 = ((center[1] - CENTER_LINEx2y2[1])+0.001)/((center[0] - CENTER_LINEx2y2[0])+0.001)
    div = (m2-m1) / (1+(m1*m2))
    return int(math.degrees(math.atan(div)))

# object_type = 0 (obstacle)
# object type = 1 (target)
def get_distance(object_type, height_of_object_pixels):
    if object_type == 0:
        return int(((OBSTACLE_HEIGHT*FOCAL_LENGTH)/((height_of_object_pixels-ERROR_VAL)/PIXEL_PER_MM))/10)
    if object_type == 1:
        return int(((TARGET_HEIGHT*FOCAL_LENGTH)/((height_of_object_pixels-ERROR_VAL)/PIXEL_PER_MM))/10)
