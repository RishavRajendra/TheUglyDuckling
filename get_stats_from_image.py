#!/usr/bin/env python
# -*- coding:utf-8 -*-
__author__ = "Rishav Rajendra"
__license__ = "MIT"
__status__ = "Development"

import cv2
import math, time
from constants import CENTER_LINEx1y1, CENTER_LINEx2y2, PIXEL_PER_MM, \
ERROR_VAL, FOCAL_LENGTH, OBSTACLE_HEIGHT, TARGET_HEIGHT, MOTHERSHIP_SIDE_HEIGHT, \
MOTHERSHIP_SLOPE_HEIGHT, CENTER_DISTANCE_UP, CENTER_DISTANCE_DOWN, CORNER_HEIGHT
from operator import itemgetter

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

# Returns angle from the mothership side
def mothership_angle(boxes_midpoint):
    m1 = ((boxes_midpoint[0][1] - boxes_midpoint[1][1])/(boxes_midpoint[0][0] - boxes_midpoint[1][0]))
    # m2 is a straight horizontal line. y2 - y1 is zero
    m2 = 0
    div = (m2-m1) / (1+(m1*m2))
    return -1*int(math.degrees(math.atan(div)))

"""
object_type = 0 (obstacle)
object type = 1 (target)
object type = 2 (side)
object type = 3 (slope)
object type = 4 (corner)

Changelog:
 -- Version 0.0.1 Rishav
    --- Mothership side has its own error val
"""
def get_distance(object_type, height_of_object_pixels):
    if object_type == 0:
        return int(((OBSTACLE_HEIGHT*FOCAL_LENGTH)/((height_of_object_pixels-ERROR_VAL)/PIXEL_PER_MM))/10)
    if object_type == 1:
        return int(((TARGET_HEIGHT*FOCAL_LENGTH)/((height_of_object_pixels-ERROR_VAL)/PIXEL_PER_MM))/10)
    if object_type == 2:
        return int(((MOTHERSHIP_SIDE_HEIGHT*FOCAL_LENGTH)/((height_of_object_pixels)/PIXEL_PER_MM))/10)
    if object_type == 3:
        return int(((MOTHERSHIP_SLOPE_HEIGHT*FOCAL_LENGTH)/((height_of_object_pixels-ERROR_VAL)/PIXEL_PER_MM))/10)
    if object_type == 4:
        return int(((CORNER_HEIGHT*FOCAL_LENGTH)/((height_of_object_pixels-ERROR_VAL)/PIXEL_PER_MM))/10)
def cam_down_distance(height_of_object_pixels):
    return int(((CAM_DOWN_TARGET_HEIGHT*FOCAL_LENGTH)/((height_of_object_pixels-ERROR_VAL)/PIXEL_PER_MM))/10)

"""
Returns the average distance of the object infront of the mothership
Mid-range IR sensor connected to the Arduino Mega 2560
Raspberry pi gets data using serial
"""
def get_sensor_data(serial):
    byteArr = b'\x08' + b'\x00' + b'\x00' + b'\x00'
    serial.write(byteArr)
    time.sleep(1)
    dist = int.from_bytes(serial.read(1),'little')
    if dist > 4 and dist <= 15:
        return dist - 3
    if dist > 15:
        return dist - 4
    else:
        return 0

# Returns [object_name, angle, inches]
def get_data(pic_q):
    processed_frame, classes, boxes, scores = pic_q.get()
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
            
            # Calculate height of the object located for distance measurements
            height_of_object_pixels = ymax - ymin
            
            """
            class 7 is the obstacle with the ping-pong ball on top
            class 8 is the side of the mothership
            class 9 is the slope of the mothership
            """
            if classes[0][i] == 7:
                inches = get_distance(0, height_of_object_pixels)
            elif classes[0][i] == 8:
                inches = get_distance(2, height_of_object_pixels)
            elif classes[0][i] == 9:
                inches = get_distance(3, height_of_object_pixels)
            angle = get_angle(processed_frame, xmin, ymin, xmax, ymax)
            print('{} detected at {}{} {} inches away. Midpoint:({},{}). Height: {}'.format(classes[0][i],angle,chr(176),inches, int(mid_x), int(mid_y), int(height_of_object_pixels)))
            result.append([int(classes[0][i]), angle, inches])
    return result

# This function returns the closest target to the robot
# Used in approach and pickUp
def get_closest_target(pic_q, mid_point=False):
    processed_frame, classes, boxes, scores = pic_q.get()
    result = []
    
    # return [0, 0, 100] if nothing is found
    if mid_point is True:
        result.append([0, 0, 100, (100, 100)])
    else:
        result.append([0, 0, 100])
    
    for i, b in enumerate(boxes[0]):
        if scores[0][i] > 0.3:
            inches = 0
            #extract pixel coordinates of detected objects
            ymin = boxes[0][i][0]*300
            xmin = boxes[0][i][1]*300
            ymax = boxes[0][i][2]*300
            xmax = boxes[0][i][3]*300

            # Calculate mid_pount of the detected object
            mid_x = (xmax + xmin) / 2
            mid_y = (ymax + ymin) / 2
            
            # Calculate height of the object located for distance measurements
            height_of_object_pixels = ymax - ymin
            
            """
            classes 1 to 6 are the six target blocks
            """
            if classes[0][i] > 0 and classes[0][i] < 7:
                inches = get_distance(1, height_of_object_pixels)
                angle = get_angle(processed_frame, xmin, ymin, xmax, ymax)
                
                if classes[0][i] == 1:
                    print('blockA detected at {}{} {} inches away. Midpoint:({},{}). Height: {}'.format(angle,chr(176),inches, int(mid_x), int(mid_y), int(height_of_object_pixels)))
                elif classes[0][i] == 2:
                    print('blockB detected at {}{} {} inches away. Midpoint:({},{}). Height: {}'.format(angle,chr(176),inches, int(mid_x), int(mid_y), int(height_of_object_pixels)))
                elif classes[0][i] == 3:
                    print('blockC detected at {}{} {} inches away. Midpoint:({},{}). Height: {}'.format(angle,chr(176),inches, int(mid_x), int(mid_y), int(height_of_object_pixels)))
                elif classes[0][i] == 4:
                    print('blockD detected at {}{} {} inches away. Midpoint:({},{}). Height: {}'.format(angle,chr(176),inches, int(mid_x), int(mid_y), int(height_of_object_pixels)))
                elif classes[0][i] == 5:
                    print('blockE detected at {}{} {} inches away. Midpoint:({},{}). Height: {}'.format(angle,chr(176),inches, int(mid_x), int(mid_y), int(height_of_object_pixels)))
                elif classes[0][i] == 6:
                    print('blockF detected at {}{} {} inches away. Midpoint:({},{}). Height: {}'.format(angle,chr(176),inches, int(mid_x), int(mid_y), int(height_of_object_pixels)))
                
                if mid_point is True:
                    result.append([int(classes[0][i]), angle, inches, (int(mid_x),int(mid_y))])
                else:
                    result.append([int(classes[0][i]), angle, inches])
    result = sorted(result, key=itemgetter(2))
    return result[0]
        
    
def get_midpoint(processed_frame, classes, boxes, scores):
    result = []
    for i, b in enumerate(boxes[0]):
        if scores[0][i] > 0.4:
            inches = 0
            #extract pixel coordinates of detected objects
            ymin = boxes[0][i][0]*300
            xmin = boxes[0][i][1]*300
            ymax = boxes[0][i][2]*300
            xmax = boxes[0][i][3]*300

            # Calculate mid_pount of the detected object
            mid_x = (xmax + xmin) / 2
            mid_y = (ymax + ymin) / 2
            
            # Calculate height of the object located for distance measurements
            height_of_object_pixels = ymax - ymin
            
            result.append([int(classes[0][i]), (int(mid_x), int(mid_y))])
    return result

def corrected_angle(angle, dist, cam_up=True):
    cd = CENTER_DISTANCE_UP if cam_up else CENTER_DISTANCE_DOWN
    sign = -1 if angle < 0 else 1
    angle = 180 - abs(angle)
    a = math.sqrt(math.pow(dist,2) + math.pow(cd, 2) - 2*dist*cd*math.cos(math.radians(angle)))
    angle_c = math.degrees(math.asin(math.sin(math.radians(angle))*dist/a))
    angle_b  = 180 -angle - angle_c
    if angle_c < angle_b:
        return math.floor(angle_c) * sign
    
    return math.floor(angle_b) * sign
