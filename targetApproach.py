from get_stats_from_image import corrected_angle, get_closest_target

# TODO: Check if the block is actually picked up
def check_pick_up(movement, pic_q):
    success = False
    movement.cam_down()
    time.sleep(3)
    target_id, angle, inches, midpoint = get_closest_target(pic_q, True)
    if target_id is not 0 and inches < 5:
        print("Pick up successful")
        success = True
    movement.cam_up()
    return success

def move_to_target(movement, angle, distance):
    movement.turn(angle)
    movement.move(fwd, distance)

def move_back_from_target(movement, angle, distance):
    movement.move(rev, distance)
    movement.turn(angle)

"""
TODO: Refine pick_up. Take care of edge cases.
Change log
    -[0.0.1] Benji
        --- Changed sleep from 2 to 1.5; lowest fps is .75 so sleeping
        --- for 1.5 seconds is the minimum delay that guarantees fresh video data
    -[0.0.2] Rishav
        --- Remove recursion. Not getting satisfying results.
"""
def pick_up(movement, pic_q):
    movement.cam_down()
    # Let the imaging catch_up
    time.sleep(3)
    # Get data from the closest object
    target_id, angle, inches, midpoint = get_closest_target(pic_q, True)
    if target_id == 0:
        print("Nothing found in pick_up")
    else:
        if midpoint[0] > 125 and midpoint[0] < 230 and midpoint[1] > 255:
            movement.pickup()
        else:
            correctedAngle = corrected_angle(angle, inches, False)
            movement.turn(-correctedAngle)

            # Get another picture as distance can get wacky at an angle
            target_id, angle, inches, midpoint = get_closest_target(pic_q, True)
            # Calculate adjusted angle from the front of the robot instead of the middle of the robot
            adjustedDistance = math.ceil(inches*0.75)
            adjustedAngle = corrected_angle(angle, adjustedDistance, False)
            
            # Angles reversed because East is 0 and we increase as we go around
            move_to_target(movement,-1*adjustedAngle, adjustedDistance)
            movement.pickup()
            move_back_from_target(movement,adjustedAngle, adjustedDistance)
            # Reverse movements
            movement.turn(correctedAngle)

    movement.cam_up()

# Moves the robot close to the target
def approach_helper(angle, distance, pic_q, movement):
    adjustedDistance = math.ceil(distance*0.9)
    adjustedAngle = corrected_angle(angle, adjustedDistance)
    # move towards the target
    # Angles reversed because East is 0 and we increase as we go around
    move_to_target(movement,-1*adjustedAngle, adjustedDistance)
    #Realign with block
    pick_up(movement, pic_q)
    # Reverse movement from the target
    move_back_from_target(movement, adjustedAngle, adjustedDistance)

# DONE: Refine approach. Take care of edge cases.
# EDGE CASE 1: obstacle is in way of target
# Potential solution: go to another connected tile
# DONE CASE 2: target not detected after two additional scans.
# DONE CASE 3: second object detected in background
# EDGE CASE 4: target at a very bad angle

"""
Change log
    -[0.0.1] Benji
        --- Changed sleep from 2 to 1.5; lowest fps is .75 so sleeping
        --- for 1.5 seconds is the minimum delay that guarantees fresh video data
    -[0.0.2] Rishav
        --- Remove recursion as results not satisfying.
"""
def approach(movement, pic_q):
    movement.cam_up()
    # Let camera catch up
    time.sleep(2)
    # Get data of the closest object
    target_id, angle, inches = get_closest_target(pic_q)
    if target_id == 0 or inches > 13:
        print("Nothing found in approach")
        # If nothing found in approach, check right and left 20 degrees. 
        movement_list = [-20, 20]
        for action in movement_list:
            movement.turn(action)
            # Let camera catch up
            time.sleep(2)
            target_id, angle, inches = get_closest_target(pic_q)
            if target_id is not 0 and inches < 14:
                print("Approach CAM_UP: [{}, {}, {}]".format(target_id, angle, inches))
                approach_helper(angle, inches, pic_q, movement)
                movement.turn(-1*action)
                break
            movement.turn(-1*action)
    else:
        print("Approach CAM_UP: [{}, {}, {}]".format(target_id, angle, inches))
        # call helper function which moves the robot
        approach_helper(angle, inches, pic_q, movement)