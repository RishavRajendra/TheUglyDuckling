#!/usr/bin/env python
# -*- coding:utf-8 -*-
__author__ = "Rishav Rajendra, Benji Lee"
__license__ = "MIT"
__status__ = "Development"

from get_stats_from_image import get_data
from main import map, follow_path
import time

def wait_for_button(buttonPin, ledPin, GPIO):
    print('Ready')
    GPIO.output(ledPin, GPIO.HIGH)
    #Prevent further code execution until button is pressed
    while GPIO.input(buttonPin) is not 0:
        pass
    GPIO.output(ledPin, GPIO.LOW)
    print('Executing') 

"""
Returns the average distance of the object infront of the mothership
Mid-range IR sensor connected to the Arduino Mega 2560
Raspberry pi gets data using serial
"""
def get_sensor_data(serial):
    byteArr = b'\x08' + b'\x00' + b'\x00' + b'\x00'
    serial.write(byteArr)
    time.sleep(1)
    left = int.from_bytes(serial.read(1),'little')
    right = int.from_bytes(serial.read(1),'little')
    return left, right
    
# TODO
def align_corner(serial):
	pass

"""
Visits given point and returns true if it is an empty space
"""
def is_point_safe(movement, pic_q, point):
	movement.set_goal(point)
	follow_path(movement, pic_q)
	map(movement, pic_q)

	return movement.grid.passable(point)


"""
Finds safe align_point and sets it
"""
def find_safe_align_points(movement, pic_q):
	# for all possible align points
	for point in movement.grid.align_points:
		if is_point_safe(movement, pic_q, point):
			# we've found a safe align point but we need to verify
			# the two connected edge tiles are also empty
			x = -1 if point[0] == 1 else 1
			y = -1 if point[1] == 1 else 1

			edges = [(point[0] + x, point[1]), (point[0], point[1] +y)]

			if is_point_safe(movement, pic_q, edge[0]) and is_point_safe(movement, pic_q, edge[1]):
				movement.align_points.append(point)
				back_dat_ass_up(movement, pic_q)

"""
Given a list of points and current point, Returns closest point
"""
def closest_point(list, current):
	closest_point = None
	prev_dist = 0
	# get x and y of current
	cx, cy = current[0], current[1]

	for point in list:
		# get x ang y of point
		px,py = point[0], point[1]

		dist = abs(cx-px) + abs(cy-py)
		
		if closest_point == None or prev_dist > dist:
			closest_point = point
			prev_dist = dist

		return closest_point

"""
Uses edge_align to straighten out our x and y angles
"""

def back_dat_ass_up(movement, pic_q):
	
	point = closest_point(movement.align_points, movement.current)

	x = -1 if point[0] == 1 else 1
	y = -1 if point[1] == 1 else 1

	edges = [(point[0] + x, point[1]), (point[0], point[1] +y)]

	for edge in edges:
		movement.set_goal(edge)
		follow_path(movement, pic_q, True)
		movement.turn(180)
		movement.edge_align()