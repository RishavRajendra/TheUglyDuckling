#!/usr/bin/env python
# -*- coding:utf-8 -*-
__author__ = "Rishav Rajendra, Benji Lee"
__license__ = "MIT"
__status__ = "Development"

from get_stats_from_image import get_data
from misc import follow_path, get_sensor_data, go_home, blink_led_twice, wait_for_contact
from get_stats_from_image import corrected_angle, two_blocks, mothership_angle, \
mothership_side_close_distance
from constants import fwd, rev, strl, strr, CONTACT_PIN
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
    movement.turn(25)
    if verify_obj(movement, pic_q, obj):
        return True 
    
    movement.turn(-25)
    if verify_obj(movement, pic_q, obj):
        return True
    
    movement.turn(-25)
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
        
def generate_access_points(side):
    tx,ty = side[0], side[1]
    c = 2 if tx < 4 else -2
    d = 2 if ty < 4 else -2

    if tx == 4:
        return [(tx, ty+d)]
    elif ty == 4:
        return [(tx + c, ty)]
    else:
        return [(tx + c, ty),(tx, ty + d)]

"""
Mapping mothership by side because we detected a side
"""

def map_by_side(movement, pic_q):
    print("Checking side")
    sx,sy = movement.grid.sides[0][0], movement.grid.sides[0][1]
    access_points = generate_access_points(movement.grid.sides[0])

    for point in access_points:
        px,py = point[0], point[1]
        
        movement.set_goal(point)
        follow_path(movement, pic_q, True)

        movement.face((sx,sy))

        if verify_obj(movement, pic_q, 8):
            cx,cy = movement.current[0], movement.current[1]
            x,y = 0,0
            if movement.facing == 90:
                y = 1
            elif movement.facing == 180:
                x = -1
            elif movement.facing == 270:
                y = -1
            elif movement.facing == 0:
                x = 1
            print("Side verified")
            print((sx,sy))
            movement.set_goal((cx+x,cy + y))
            follow_path(movement, pic_q, True)
            movement.map_mothership((sx,sy))
            return

    print("Couldn't locate side")
    go_home(movement, pic_q)

def map_by_slope(movement, pic_q):
    print("Checking slope")
    
    # If two slopes then we know where a side is
    if len(movement.grid.slopes) == 2:
        movement.grid.sides.append(movement.grid.slopes[0])
        map_by_side(movement, pic_q)
        return
    else:
        sx,sy = movement.grid.slopes[0][0], movement.grid.slopes[0][1]
        guesses = generate_guesses(movement.grid.slopes[0])

        for guess in guesses:
            movement.grid.sides.clear()
            movement.grid.sides.append(guess)
            map_by_side(movement, pic_q)
            if movement.grid.mothership:
                return

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
            movement.turn(corrected_angle(initial_angle, initial_camera_distance))
            
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

    # TODO
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

def drop_right_spot_helper(movement, blocks_located, target_id):
    for blocks in blocks_located:
        if blocks[0] == target_id:
            angle = 90 - blocks[1]
            distance = blocks[2]

            # sin(theta) = opposite / hypotenuse
            distance_to_streff = distance * math.sin(abs(angle))
            if angle < 0:
                movement.move(strl, distance_to_streff -1)
            else:
                movement.move(strr, distance_to_streff -1)

    # If the target id is not in the located blocks, move w.r.t what is detected
    for blocks in blocks_located:
        diff = abs(blocks[0] - target_id)
        if diff <= 2:
            # One drop point is 3 inches wide
            distance_to_streff = (3*diff) - 1
        else:
            print("NOT ON THE RIGHT SIDE YOU IDIOT")

def drop_right_spot(target_id, pic_q, movement):
    movement.cam_up()

    time.sleep(3)

    blocks_located = two_blocks(pic_q)

    if len(blocks_located) is not 0:
        drop_right_spot_helper(movement, blocks_located, target_id)
    else:
        i = 0
        # None of the obstacles detected
        for _ in range(5):
            i = i + 1
            movement.move(rev, 1)
            time.sleep(2)
            blocks_located = two_blocks(pic_q)
            if len(blocks_located) is not 0:
                drop_right_spot_helper(movement, blocks_located, target_id)
                break

        movement.move(fwd, i)

"""
You can rewrite this with movement.get_mothership_angle(), movement.get_side_angle(), and  movement.get_access_dist()
"""
def mothership_drop(distance_from_access, angle_from_access, mothership_orient, block_id, movement, serial, pic_q, GPIO):
    # Get to the mothership
    movement.turn(-1*angle_from_access)
    movement.move(fwd, distance_from_access)
    movement.turn(-1*mothership_orient)

    drop_right_spot(block_id, pic_q, movement)

    i = 0
    while GPIO.input(CONTACT_PIN) is not 0 and i <= 5:
        movement.move(fwd, 1)
        i = i + 1
    
    movement.drop()
    
    movement.turn(mothership_orient)
    movement.move(rev, distance_from_access)
    movement.pickup()
    movement.turn(angle_from_access)
    
"""
Assumes we're already in access point. Goes to the other side of the mothership
"""

def approach_other_side(movement, pic_q):
    # approaches the access point's side
    movement.turn(-1*movement.get_mothership_angle())
    movement.move(fwd, movement.get_access_dist()-2)
    movement.turn(-1*movement.get_side_angle())
    
    cx,cy = movement.current[0], movement.current[1]
    # determine which direction is safest to attempt to go around

    if movement.facing == 90:
        sign = 1 if cx > 4 else -1
    elif movement.facing == 180:
        sign = 1 if cy > 4 else -1
    elif movement.facing == 270:
        sign = 1 if cx < 4 else -1
    elif movement.facing == 0:
        sign = 1 if cy < 4 else -1

    # now go around
    movement.turn(90*sign)
    movement.move(fwd, 16) #This distance needs to be checked
    movement.turn(90*sign*-1)
    movement.move(fwd, 24) #This distance needs to be checked
    movement.turn(90*sign*-1)
    movement.move(fwd, 16) #This distance needs to be checked
    movement.turn(90*sign*-1)
    #We should be ready for drop off now


"""
Reverses our approach other side . Assumes we've already approached other side
"""

def rev_other_side(movement, pic_q):
    cx,cy = movement.current[0], movement.current[1]

    # determine what direction to come back from
    if movement.facing == 90:
        sign = 1 if cx > 4 else -1
    elif movement.facing == 180:
        sign = 1 if cy > 4 else -1
    elif movement.facing == 270:
        sign = 1 if cx < 4 else -1
    elif movement.facing == 0:
        sign = 1 if cy < 4 else -1

    # move back to other side
    movement.turn(90*sign)
    movement.move(fwd, 16) #This distance needs to be checked
    movement.turn(90*sign*-1)
    movement.move(fwd, 24) #This distance needs to be checked
    movement.turn(90*sign*-1)
    movement.move(fwd, 16) #This distance needs to be checked
    movement.turn(90*sign*-1)
    
    # rev from the access point's side
    movement.turn(movement.get_side_angle())
    movement.move(rev, movement.get_access_dist()-2)
    movement.turn(movement.get_mothership_angle())
