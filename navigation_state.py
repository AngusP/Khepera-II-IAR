import constants

class Navigation_State:
    follow_left = False
    follow_right = False
    boredom_counter = 0
    boredom_turn_counter = 0
    speed_left = constants.CONST_SPEED
    speed_right = constants.CONST_SPEED
    system_state = constants.STATE_DRIVE_FORWARD
    
    