#!/usr/bin/env python
# -*- coding:utf-8 -*-
__author__ = "Rishav Rajendra, Benji Lee"
__license__ = "MIT"
__status__ = "Development"

from get_stats_from_image import get_data
from misc import follow_path, get_sensor_data, go_home, blink_led_twice
from get_stats_from_image import corrected_angle, two_blocks, mothership_angle, \
mothership_side_close_distance
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
Generate guesses as to where the side might be based on slope location
"""
def generate_guesses(slope):
    tx,ty = slope[0], slope[1]
    c = 1 if tx > 4 else -1
    d = 1 if ty > 4 else -1

    if tx == 4:
        return [(tx + 1, ty+d),(tx -1, ty + d)]
    elif ty == 4:
        return [(tx + c, ty+1),(tx +c, ty - 1)]
    else:
        return [(tx + c, ty),(tx, ty + d)]

"""
Mapping mothership by side because we detected a side
"""
def map_by_side(movement, pic_q):
    print("Checking side")
    # Move to initial mapped side or updated side
    movement.set_goal(movement.grid.sides[0])
    follow_path(movement, pic_q)
    prev_side = movement.grid.sides[0]
    # Verify location
    if verify_obj(movement, pic_q, 8):
        print("Side Verified")
        # remap side now that we're closer
        
        
        current = movement.grid.sides[0]
        print('Side location adjusted')
        print("Previous side location was: {}".format(prev_side))
        print("Current side location is: {}".format(current))

        # If it was in the correct location
        if prev_side[0] == current[0] and prev_side[1] == current[1]:
            # we can map all the other pieces and current location is access point
            print("Success! Now work on the logic for placing the other pieces")
            movement.map_mothership(prev_side)
            # approach mothership
        else:
            # map by side again
            map_by_side(movement, pic_q)
        
    # else if we locate it.
    elif locate_obj(movement, pic_q, 8):
        # remap side now that we're closer
    
        current = movement.grid.sides[0]
        print('Side location adjusted')
        print("Previous side location was: {}".format(prev_side))
        print("Current side location is: {}".format(current))

        # If it was in the correct location
        if prev_side[0] == current[0] and prev_side[1] == current[1]:
            # we can map all the other pieces and current location is access point
            movement.map_mothership(prev_side)
            # approach mothership
        else:
            # map by side again
            map_by_side(movement, pic_q)

    # else go home - you're drunk - try again later
    else:
        print("Going Home")
        go_home(movement, pic_q)  
        
"""
Mapping mothership by slope because we didn't find a side
"""
def map_by_slope(movement, pic_q):
    print("Checking slope")
    # Move to initial mapped slope or updated slope
    movement.set_goal(movement.grid.slopes[0])
    follow_path(movement, pic_q)
    prev_slope = movement.goal
    # Verify the slope's location
    if verify_obj(movement, pic_q, 9):
        current = movement.grid.slopes[0]
        print('Slope location adjusted')
        print("Previous slope location was: ", prev_slope)
        print("Current slope location is: ", current)

        if movement.grid.sides:
                map_by_side(movement, pic_q)
                return
        # if it's in the correct location
        if prev_slope[0] == current[0] and prev_slope[1] == current[1]:
            # create two guesses to map by side from
            slope = movement.grid.slopes[0]
            # we try to map by side for both possibilities
            tx, ty = slope[0], slope[1]
            
            guesses = generate_guesses(slope)
            
            print("Guesses are: ", guesses)
                
            for guess in guesses:
                movement.grid.sides.clear()
                movement.grid.sides.append(guess)
                map_by_side(movement, pic_q)
                # we break if mothership has anything in it - we only add to mothership 
                # list if map by side was a success
                if movement.grid.mothership:
                    break

        else:
            # map by slope again
            map_by_slope(movement, pic_q)

    # Else if we locate the slope
    elif locate_obj(movement, pic_q, 9):
        # remap side now that we're closer
        
        current = movement.grid.slopes[0]
        print('Slope location adjusted')
        print("Previous slope location was: ", prev_slope)
        print("Current slope location is: ", current)

        if movement.grid.sides:
                map_by_side(movement, pic_q)
                return

        # if it's in the correct location
        if prev_slope[0] == current[0] and prev_slope[1] == current[1]:
            # create two guesses to map by side from
            slope = movement.grid.slopes[0]
            # we try to map by side for both possibilities
            tx, ty = slope[0], slope[1]

            guesses = generate_guesses(slope)
            
            if movement.grid.sides:
                map_by_side(movement,pic_q)
            else:
                for guess in guesses:
                    movement.grid.sides.clear()
                    movement.grid.sides.append(guess)
                    map_by_side(movement, pic_q)
                    # we break if mothership has anything in it - we only add to mothership 
                    # list if map by side was a success
                    if movement.grid.mothership:
                        break
        else:
            # map by slope again
            map_by_slope(movement, pic_q)
    # else go home - you're drunk - try again later
    else:

        if movement.grid.sides:
                map_by_side(movement, pic_q)
                return
        print("Mapping Failed")
        go_home(movement, pic_q)

def map_mothership(movement, pic_q):
    #movement.drop()
    if movement.grid.sides:

        map_by_side(movement, pic_q)
    elif movement.grid.slopes:
        map_by_slope(movement, pic_q)
    #movement.reset_servo()

# Two mid-range IR sensors mounted in the front of the robot facing forward
# Returns the avg distance from both sensors
def sensor_distance(serial):
	# Get current distance with IR sensor for validation
    left_sensor_dist, right_sensor_dist = get_sensor_data(serial)
    if (left_sensor_dist - right_sensor_dist) > 5:
        distance_from_sensor = right_sensor_dist
    elif (right_sensor_dist - left_sensor_dist) > 5:
        distance_from_sensor = left_sensor_dist
    else:
        distance_from_sensor = (right_sensor_dist + left_sensor_dist) / 2
    print("Distance from sensor: Left[{}] Right[{}]".format(left_sensor_dist, right_sensor_dist))
    return math.ceil(distance_from_sensor)

def approach_mothership_side_helper(camera_distance, distance_from_sensor, angle, pic_q, serial, movement, GPIO):
    # Move towards the side of the mothership
    if camera_distance == 0:
        distance_to_move = distance_from_sensor - 1
    else:
        distance_to_move = int((distance_from_sensor+camera_distance)/2)-1

    movement.move(fwd, distance_to_move)
    # Get the angle of the mothership. Found not found on first try, move left and right one inch
    side_angle = mothership_side_angle(movement, pic_q, 1, serial, GPIO)
            
    # Try again if angle could not be found
    # This generally happens when robot is too far from the side
    if side_angle is False:
        side_angle = mothership_side_angle(movement, pic_q, 2, serial, GPIO)
    
    if side_angle is False:
        movement.move(fwd, 1)
        side_angle = mothership_side_angle(movement, pic_q, 2, serial, GPIO)
        movement.move(rev, 1)

    if side_angle is False:
        # After the second try, give up for now
        side_angle = 0

    # Reverse movements
    movement.move(rev, distance_to_move)
    # Side should only be detected once
    #movement.turn(corrected_angle(angle, camera_distance))
    print("SIDE ANGLE: {}".format(side_angle))
                    
    return side_angle

""" 
Approach mothership from close
Assuming that the robot is atleast pointed towards the mothership
Gets close the the mothership and calls mothership side angle
"""
def approach_mothership_side(movement, pic_q, serial, GPIO):
    time.sleep(3)  # Let camera catch up
    initial_camera_distance, initial_angle = mothership_side_close_distance(pic_q) # Grab picture data
    print("Camera Distance: {}, angle: {}".format(initial_camera_distance, initial_angle))
    
    if initial_camera_distance is not 100:
        movement.turn(-corrected_angle(initial_angle, initial_camera_distance))

        # Get distance from the sensors
        distance_from_sensor = sensor_distance(serial)
        print('Distance from sensor: {}'.format(distance_from_sensor))

        # If the difference of sensor distance and camera distance is greater that 5,
        # Something is wrong
        print(abs(distance_from_sensor - initial_camera_distance))
        if abs(distance_from_sensor - initial_camera_distance) <= 5:
            print("-------------BEST CASE-------------")
            print("Mothership sensor validation passed")
            
            side_angle = approach_mothership_side_helper(initial_camera_distance, distance_from_sensor, initial_angle, pic_q, serial, movement, GPIO)
            movement.turn(corrected_angle(initial_angle, initial_camera_distance))
            
            return [corrected_angle(initial_angle, initial_camera_distance),int((distance_from_sensor+initial_camera_distance)/2),side_angle]
        else:
            print("Mothership sensor validation Failed")
            print("-----------WORST CASE----------")
            
            # Just rely on sensors if everything else fails
            side_angle = approach_mothership_side_helper(0, distance_from_sensor, initial_angle, pic_q, serial, movement, GPIO)
            #movement.turn(corrected_angle(initial_angle, initial_camera_distance))
            
            return [corrected_angle(initial_angle, initial_camera_distance),int((distance_from_sensor+initial_camera_distance)/2),side_angle]
                    
        movement.turn(corrected_angle(initial_angle, initial_camera_distance))
        # WORST WORST CASE: Side angle is 0
        return [corrected_angle(initial_angle, initial_camera_distance),int((distance_from_sensor+initial_camera_distance)/2),0]
    else:
        # TODO: Handle edge cases
        print("Nothing Detected in approach")
        # TODO
        # Lets back up 5 inches to give up more of a perspective
        movement.move(rev, 5)
            
        # Let camera catch up
        time.sleep(2)
        distance_from_sensor = sensor_distance(serial)
        print('Distance from sensor: {}'.format(distance_from_sensor))

        camera_distance, angle = mothership_side_close_distance(pic_q) # Grab picture data
        if camera_distance is not 100:
            if abs(distance_from_sensor - camera_distance) <= 5:
                print("--------------Test Again-------------")
                print("Mothership sensor validation passed")
                    
                side_angle = approach_mothership_side_helper(camera_distance, distance_from_sensor, angle, pic_q, serial, movement, GPIO)
                    
                #movement.turn(corrected_angle(initial_angle, initial_camera_distance))
                movement.move(fwd, 5)
                    
                return [corrected_angle(initial_angle, initial_camera_distance),int((distance_from_sensor+initial_camera_distance)/2),side_angle]
        movement.move(fwd, 5)
        return [corrected_angle(initial_angle, initial_camera_distance),int((distance_from_sensor+initial_camera_distance)/2),0]

"""
Turns the angle it believes the mothership is at from the front of the robot
"""
def mothership_side_angle(movement, pic_q, side_move_distance, serial, GPIO):
    # Camera down to look at the letters inside the mothership
    movement.cam_down()
    time.sleep(2)
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

                # Show indication that angle has been detected
                blink_led_twice(GPIO)
                print("Worst case")
                print("Angle detection successful")
                print("Side angle is: {}".format(side_angle))
                
                # Reverse movements
                if action[0] is strr:
                    movement.move(strl, action[1])
                else:
                    movement.move(strr, action[1])
                #movement.turn(-side_angle)
                return side_angle
            print("Down here")
            if action[0] is strr:
                movement.move(strl, action[1])
            else:
                movement.move(strr, action[1])
    else:
        # Two blocks detected on the first try
        #Get angle of the two blocks w.r.t to front of the robot
        print("BEST CASE")
        print("Angle detection successful")
        
        # Show indication that angle has been detected
        blink_led_twice(GPIO)
        
        side_angle = math.ceil(mothership_angle([blocks_in_mothership[0][3], blocks_in_mothership[1][3]])*1.18)
        
        return side_angle
        
    # If it does not detect two blocks, return None
    print('---------------Angle could not be detected--------------')
    return False

def mothership_drop(distance_from_access, angle_from_access, mothership_orient, block_id, movement, serial, pic_q):
    # Get to the mothership
    movement.turn(-1*angle_from_access)
    movement.move(fwd, distance_from_access)
    movement.turn(-1*mothership_orient)
    
    left_sensor, right_sensor = get_sensor_data(serial)
    
    for _ in range(5):
        if left_sensor > 1 and right_sensor > 1:
            movement.move(fwd, 1)
        else:
            break
    
    movement.drop()
    movement.pickup()
    
    movement.turn(mothership_orient)
    movement.move(rev, distance_from_access)
    movement.turn(angle_from_access)
    
