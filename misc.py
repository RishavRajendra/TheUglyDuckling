#!/usr/bin/env python
# -*- coding:utf-8 -*-
__author__ = "Rishav Rajendra, Benji Lee"
__license__ = "MIT"
__status__ = "Development"

from get_stats_from_image import get_data
from constants import LEDPIN, BUTTONPIN, CONTACT_PIN, fwd, rev
from targetApproach import approach, approach_obstacle
import time

def wait_for_button(GPIO):
    GPIO.output(LEDPIN, GPIO.HIGH)
    #Prevent further code execution until button is pressed
    while GPIO.input(BUTTONPIN) is not 0:
        pass
    GPIO.output(LEDPIN, GPIO.LOW)

def wait_for_contact(GPIO):
	print("Waiting for contact")
	while GPIO.input(CONTACT_PIN) is not 0:
		pass
	print("THERE IS CONTACT")

def blink_led_twice(GPIO):
    GPIO.output(LEDPIN, GPIO.HIGH)
    time.sleep(.2)
    GPIO.output(LEDPIN, GPIO.LOW)
    time.sleep(.2)
    GPIO.output(LEDPIN, GPIO.HIGH)
    time.sleep(.2)
    GPIO.output(LEDPIN, GPIO.LOW)

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
	
	point = closest_point(movement.grid.edges, movement.current)
	print("Closest align point is: ", point)

	movement.set_goal(point)
	follow_path(movement, pic_q, True)

	px, py = point[0], point[1]

	if px == 0:
		angle = 360 - movement.facing if movement.facing >180 else 0 - movement.facing
	elif px == 7:
		angle = 180 - movement.facing
	elif py == 0:
		angle = 90 - movement.facing
	else:
		angle = 270 - movement.facing if not movement.facing == 0 else 270 - 360 
	
	movement.turn(angle)
	movement.edge_align()
	
		

"""
Change log
    -[0.0.1] Benji
        --- Changed sleep from 2 to 1.5; lowest fps is .75 so sleeping
        --- for 1.5 seconds is the minimum delay that guarantees fresh video data
"""
def map(movement, pic_q, beginning=True):
    movement.cam_up()
    print(movement.facing)
    time.sleep(3)
    object_stats = get_data(pic_q)
    for stat in object_stats:
        obj_type = stat[0]
        angle = stat[1]
        dist = stat[2]
        print(obj_type, angle, dist)
        if obj_type == 9:
            dist = dist + 3
        movement.map(obj_type, angle, dist)
    # experimental
    """
    if movement.is_mothership():
    	movement.map(8, 0, 10):
		"""
		
"""
Change Log
    [0.0.1]
        --- Added parameter to allow including goal in path
"""
def follow_path(movement, pic_q, include_goal=False, map_as_we_go=True):
	# if we are alreadywhere we want to be then we return
	if movement.goal == movement.current:
		return
	# find path without include goal. If we are including goal
	# we will add it as the last move
	movement.find_path()

	# now we execute the damn thing
	while movement.path:
		print(movement.path)
		movement.follow_next_step()
		if map_as_we_go:
			map(movement, pic_q)
		# we're mapping as we go by default so after each move we
		# check if there is a blocked tile in our path
		for move in movement.path:
			# if we find the path is blocked then we generate a new
			# one and keep going
			if not movement.grid.passable(move):
				movement.path.clear()
				movement.find_path()
	# After following the path face the goal
	if not movement.goal == movement.current:
		movement.face(goal)
	# Now that we've followed the path
	# if we're including the goal we can go to it now
	if include_goal:
		movement.find_path(include_goal)
		# check if an object is in that last space
		if movement.path[0] in movement.grid.obstacles:
			old_goal = (movement.goal[0], movement.goal[1])
			# kill the damn thing
			kill_object(movement, pic_q)
			movement.grid.obstacles.remove(old_goal)
			movement.set_goal(old_goal)
			movement.find_path()
		# if target in the way move target to new location
		elif movement.path[0] in movement.grid.targets:
			old_goal = (movement.goal[0], movement.goal[1])
			relocate_target(movement, pic_q)
			movement.set_goal(old_goal)
			movement.find_path()
		movement.follow_next_step()
"""
def follow_path(movement, pic_q, include_goal=False, map_as_we_go=True):
    if movement.goal == movement.current:
    	return
    movement.find_path(include_goal)
    while movement.path:
    		
    	# check if object in last space that we want to move in.
    	if len(movement.path) == 1 and include_goal:
    		# if object is obstacle
   			if movement.path[0] in movement.grid.obstacles:
   				old_goal = (movement.goal[0],movement.goal[1])
   				# check if obstacle in way
   				kill_object(movement, pic_q)
   				movement.grid.obstacles.remove(old_goal)
    			movement.set_goal(old_goal)
    			movement.find_path(include_goal)
    			
    		# if object is target
    		elif movement.path[0] in movement.grid.targets:
    			# move target to another area and update the location
    			old_goal = (movement.goal[0],movement.goal[1])
   				relocate_target(movement, pic_q)
   				movement.set_goal(old_goal)
   				movement.find_path(include_goal)

        print(movement.path)
        movement.follow_next_step()
        if map_as_we_go:
        	map(movement, pic_q)
        for move in movement.path:
            if not movement.grid.passable(move):
                movement.path.clear()
                movement.find_path(include_goal)
    if not movement.goal == movement.current:
        movement.face(movement.goal)
"""
# relocate a target block and update the targets list
def relocate_target(movement, pic_q):
	cx,cy = movement.current[0], movement.current[1]
	x,y = 0,0
	
	if movement.facing == 90:
		y = -1
	elif movement.facing == 180:
		x = 1
	elif movement.facing == 270:
		y = 1
	elif movement.facing == 0:
		x = 1

	approach(movement,pic_q)
	# remove the target's block
	movement.grid.targets.remove(movement.goal)
	movement.turn(180)
	movement.move(fwd ,6)
	movement.drop()
	movement.movement(rev, 6)
	movement.grid.targets.append((cx+x,cy+y))


# Called at the start of the round. 
# Maps relative locations to obstacles and mothership from start point
def begin_round(movement, pic_q):
    for _ in range(8):
        movement.turn(45)
        map(movement, pic_q, True)

"""
Go to Home square
"""
def go_home(movement, pic_q):
    if not movement.goal == movement.current:
        movement.path.clear()
        movement.set_goal((4,4))
        follow_path(movement, pic_q, True) 

"""
You forced my hand Layfette
"""
def kill_object():
	approach_obstacle(movement, pic_q)
	back_dat_ass_up(movement, pic_q)
	movement.turn(180)
	movement.drop()
	