#
#     - - - -  I A R  - - - -
#
#  s1311631         Angus Pearson
#  s1346981    Jevgenij Zubovskij
#


import constants

class Navigation_State:

    #control loop epochs that the robot was geting "bored" for
    boredom_counter = 0
    #is the robot done turning from boredom causing wall / object
    boredom_turn_counter = 0
    #constants speed initla conditions
    speed_l = constants.CONST_SPEED
    speed_r = constants.CONST_SPEED
    #initially drive forward
    system_state = constants.STATE_DRIVE_FORWARD
    #IR sensor readings
    dist = [0]*8

