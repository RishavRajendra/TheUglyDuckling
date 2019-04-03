#!/usr/bin/env python
# -*- coding:utf-8 -*-
__author__ = "Rishav Rajendra, Benji Lee"
__license__ = "MIT"
__status__ = "Development"

from get_stats_from_image import get_data
from misc import follow_path, get_sensor_data, go_home
from get_stats_from_image import corrected_angle, two_blocks, mothership_angle
from constants import fwd, rev, rotl, rotr, strl, strr
import time, math

"""
Return true if obj is in our field of vision
else change goal and return False
"""
def verify_obj(movement, pic_q, obj):
    time.sleep(3)
    object_stats = get_data(pic_q)

    for stats in object_stats:
        if stats[0] == obj:
            o_type = stats[0]
            angle = stats[1]
            dist = stats[2]
            if obj == 9:
                movement.grid.slopes.clear()
                dist = dist + 3
            if obj == 8:
                movement.grid.sides.clear()
            movement.map(o_type, angle, dist)
            return True
    
    # If we can't verify it through imaging then
    # we use the sensors
    return movement.is_mothership()
    
def locate_obj(movement, pic_q, obj):
    movement.turn(45)
    if verify_obj(movement, pic_q, obj):
        return True 
    
    movement.turn(-45)
    if verify_obj(movement, pic_q, obj):
        return True
    
    movement.turn(-45)
    if verify_obj(movement, pic_q, obj):
        return True
            
    return False

"""
Mapping mothership by side because we detected a side
"""
def map_by_side(movement, pic_q, log):
    log.info("Checking side")
    # Move to initial mapped side or updated side
    movement.set_goal(movement.grid.sides[0])
    follow_path(movement, pic_q)
    prev_side = movement.grid.sides[0]
    # Verify location
    if verify_obj(movement, pic_q, 8):
        log.info("Side Verified")
        # remap side now that we're closer
        
        
        current = movement.grid.sides[0]
        log.info('Side location adjusted')
        log.debug("Previous side location was: {}".format(prev_side))
        log.debug("Current side location is: {}".format(current))

        # If it was in the correct location
        if prev_side[0] == current[0] and prev_side[1] == current[1]:
            # we can map all the other pieces and current location is access point
            log.info("Success! Now work on the logic for placing the other pieces")
            movement.map_mothership(prev_side)
            # approach mothership
        else:
            # map by side again
            map_by_side(movement, pic_q, log)
        
    # else if we locate it.
    elif locate_obj(movement, pic_q, 8):
        # remap side now that we're closer
    
        current = movement.grid.sides[0]
        log.info('Side location adjusted')
        log.debug("Previous side location was: {}".format(prev_side))
        log.debug("Current side location is: {}".format(current))

        # If it was in the correct location
        if prev_side[0] == current[0] and prev_side[1] == current[1]:
            # we can map all the other pieces and current location is access point
            movement.map_mothership(prev_side)
            # approach mothership
        else:
            # map by side again
            map_by_side(movement, pic_q, log)

    # else go home - you're drunk - try again later
    else:
        log.info("Going Home")
        go_home(movement, pic_q)  
        
"""
Mapping mothership by slope because we didn't find a side
"""
def map_by_slope(movement, pic_q, log):
    log.info("Checking slope")
    # Move to initial mapped slope or updated slope
    movement.set_goal(movement.grid.slopes[0])
    follow_path(movement, pic_q)
    prev_slope = movement.goal
    # Verify the slope's location
    if verify_obj(movement, pic_q, 9):
        current = movement.grid.slopes[0]
        log.info('Slope location adjusted')
        log.debug("Previous slope location was: ", prev_slope)
        log.dubug("Current slope location is: ", current)

        # if it's in the correct location
        if prev_slope[0] == current[0] and prev_slope[1] == current[1]:
            # create two guesses to map by side from
            slope = movement.grid.slopes[0]
            # we try to map by side for both possibilities
            tx, ty = slope[0], slope[1]

            c = 1 if tx > 4 else -1
            d = 1 if ty > 4 else -1
            guesses = [(tx + c, ty),(tx, ty + d)]
            log.debug("Guesses are: ", guesses)
            
            if movement.grid.sides:
                map_by_side(movement, pic_q, log)
            else:
                
                for guess in guesses:
                    movement.grid.sides.append(guess)
                    map_by_side(movement, pic_q, log)
                    # we break if mothership has anything in it - we only add to mothership 
                    # list if map by side was a success
                    if movement.grid.mothership:
                        break

        else:
            # map by slope again
            map_by_slope(movement, pic_q, log)

    # Else if we locate the slope
    elif locate_obj(movement, pic_q, 9):
        # remap side now that we're closer
        
        current = movement.grid.slopes[0]
        log.info('Slope location adjusted')
        log.debug("Previous slope location was: ", prev_slope)
        log.dubug("Current slope location is: ", current)

        # if it's in the correct location
        if prev_slope[0] == current[0] and prev_slope[1] == current[1]:
            # create two guesses to map by side from
            slope = movement.grid.slopes[0]
            # we try to map by side for both possibilities
            tx, ty = slope[0], slope[1]

            c = 1 if tx > 4 else -1
            d = 1 if ty > 4 else -1
            guesses = [(tx + c, ty),(tx, ty + d)]
            
            if movement.grid.sides:
                map_by_side(movement,pic_q, log)
            else:
                for guess in guesses:
                    movement.grid.sides.append(guess)
                    map_by_side(movement, pic_q, log)
                    # we break if mothership has anything in it - we only add to mothership 
                    # list if map by side was a success
                    if movement.grid.mothership:
                        break
        else:
            # map by slope again
            map_by_slope(movement, pic_q, log)
    # else go home - you're drunk - try again later
    else:
        log.info("Mapping Failed")
        go_home(movement, pic_q)

def map_mothership(movement, pic_q, log):
    #movement.drop()
    if movement.grid.sides:
        map_by_side(movement, pic_q, log)
    else:
        map_by_slope(movement, pic_q, log)
    #movement.reset_servo()

# Two mid-range IR sensors mounted in the front of the robot facing forward
# Returns the avg distance from both sensors
def sensor_distance(serial, log):
	# Get current distance with IR sensor for validation
    left_sensor_dist, right_sensor_dist = get_sensor_data(serial)
    if (left_sensor_dist - right_sensor_dist) > 5:
        distance_from_sensor = right_sensor_dist
    elif (right_sensor_dist - left_sensor_dist) > 5:
        distance_from_sensor = left_sensor_dist
    else:
        distance_from_sensor = (right_sensor_dist + left_sensor_dist) / 2
    log.debug("Distance from sensor: Left[{}] Right[{}]".format(left_sensor_dist, right_sensor_dist))
    #return distance_from_sensor
    # Right sensor bad
    return left_sensor_dist

""" 
Approach mothership from close
Assuming that the robot is atleast pointed towards the mothership
Gets close the the mothership and calls mothership side angle
"""
def approach_mothership_side(movement, pic_q, serial, log):
    time.sleep(3)  # Let camera catch up
    object_stats = get_data(pic_q) # Grab picture data
    
    print(object_stats)
    
    if object_stats:
        for stats in object_stats:
            if stats[0] == 8:
                movement.turn(-corrected_angle(stats[1], stats[2]))

                # Get distance from the sensors
                distance_from_sensor = sensor_distance(serial)
                log.debug('Distance from sensor: {}'.format(distance_from_sensor))

                # If the difference of sensor distance and camera distance is greater that 5,
                # Something is wrong
                if abs(distance_from_sensor - stats[2]) <= 5:
                    log.info("----------------Mothership sensor validation passed----------------")
                    # Move towards the side of the mothership
                    movement.move(fwd, int((distance_from_sensor+stats[2])/2)-1)
                    # Get the angle of the mothership. Found not found on first try, move left and right one inch
                    side_angle = mothership_side_angle(movement, pic_q, 1, log)
                    # Reverse movements
                    movement.move(rev, int((distance_from_sensor+stats[2])/2)-1)
                    # Side should only be detected once
                    movement.turn(corrected_angle(stats[1], stats[2]))
                    
                    if side_angle is not None:
                        return [corrected_angle(stats[1], stats[2]),int((distance_from_sensor+stats[2])/2),side_angle]
                else:
                    log.info("-------------Validation Failed-----------------")
                    break
                    # TODO: Handle edge cases
                    #approach_mothership_side(movement, pic_q, serial)
            else:
                print("Side not detected")
    else:
        # TODO: Handle edge cases
        print("Nothing Detected in approach")

"""
Turns the angle it believes the mothership is at from the front of the robot
"""
def mothership_side_angle(movement, pic_q, side_move_distance, log):
    # Camera down to look at the letters inside the mothership
    movement.cam_down()
    # Get the closest two blocks from the camera
    blocks_in_mothership = two_blocks(pic_q)

    # Checks current location, left and right for the two blocks inside the mothership
    movement_list = [[strl, side_move_distance], [strr, side_move_distance]]

    # If less than two blocks are detected, move robot left and right
    if len(blocks_in_mothership) < 2:
        for action in movement_list:
            movement.move(action[0], action[1])
            time.sleep(3)   # Give the camera time to catch up
            blocks_in_mothership = two_blocks(pic_q) # Take another picture after moving
            if len(blocks_in_mothership) > 1:
                #Get angle of the two blocks w.r.t to front of the robot
                side_angle = math.ceil(mothership_angle([blocks_in_mothership[0][3], blocks_in_mothership[1][3]])*1.18)
                log.info("Angle detection successful")
                log.debug("Side angle is: {}".format(side_angle))
                # Reverse movements
                if action[0] is strr:
                    movement.move(strl, action[1])
                else:
                    movement.move(strr, action[1])
                #movement.turn(-side_angle)
                return side_angle
            if action[0] is strr:
                movement.move(strl, action[1])
            else:
                movement.move(strr, action[1])
    else:
        # Two blocks detected on the first try
        #Get angle of the two blocks w.r.t to front of the robot
        log.info("Angle detection successful")
        side_angle = math.ceil(mothership_angle([blocks_in_mothership[0][3], blocks_in_mothership[1][3]])*1.18)
        log.debug("Side angle is: {}".format(side_angle))
        return side_angle
    # If it does not detect two blocks, return None
    log.info('---------------Angle could not be detected--------------')
    return None