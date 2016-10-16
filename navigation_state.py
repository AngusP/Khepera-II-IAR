import constants

class Navigation_State:

    boredom_counter = 0
    boredom_turn_counter = 0
    speed_l = constants.CONST_SPEED
    speed_r = constants.CONST_SPEED
    system_state = constants.STATE_DRIVE_FORWARD
    dist = [0]*8


    bug_control = True
