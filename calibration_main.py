#!/usr/bin/env python

#
#     - - - -  I A R  - - - -
#
#  s1311631         Angus Pearson
#  s1346981    Jevgenij Zubovskij
#

from __future__ import print_function
from comms import Comms
from odometry_algorithm_1 import Odometry_Algorithm_1
from odometry_algorithm_2 import Odometry_Algorithm_2
from odometry_state import Odometry_State


import constants
import sys
import getopt      # CLI Option Parsing
import whiptail    # Simplest kinda-GUI thing
import time
import math
import matplotlib.pyplot as plt
from data import DataStore

namebadge = " -- IAR C&C -- "
helptext = str(sys.argv[0]) + ' -p <serial port> -b <baud rate> -t <timeout> -s <server hostname>'

wt = whiptail.Whiptail(title=namebadge)

# check if we are stuck
def is_stuck(dist):
    #check if we are scraping on the sides
    stuck_cone_left  = dist[1] > constants.CONST_WALL_DIST * 1.2 # multiple of 1.2 as 1.0 is handled by following
    stuck_cone_right = dist[4] > constants.CONST_WALL_DIST * 1.2  # multiple of 1.2 as 1.0 is handled by following
    #check if we are about to be stuck in the front
    stuck_cone_front = dist[2] > constants.CONST_WALL_DIST*0.7 or dist[3]  > constants.CONST_WALL_DIST*0.7

    return stuck_cone_left or stuck_cone_right or stuck_cone_front 


# check if we "see" the left wall and it is closer than the one on the right
def should_follow_left_wall(dist):
        
    return dist[0] > constants.CONST_WALL_DIST and dist[0] > dist[5]
        

# check if we "see" the right wall and it is closer than the one on the left
def should_follow_right_wall(dist):

    return dist[5] > constants.CONST_WALL_DIST and dist[5] > dist[0]


# check if we are over the distance threshold w.r.t. object on the left
def too_close_to_left(dist):

    distance_close   = dist[0] > constants.CONST_WALL_DIST
    return distance_close 


# check if we are over the distance threshold w.r.t. object on the right
def too_close_to_right(dist):

    distance_close   = dist[5] > constants.CONST_WALL_DIST
    return distance_close 



# check if we are under the distance threshold w.r.t. object on the right
def is_away_from_right(dist):

    wall_in_range = dist[5] < constants.CONST_WALL_DIST - constants.CONST_WALL_OFFSET
    return wall_in_range  


# check if we are under the distance threshold w.r.t. object on the left
def is_away_from_left(dist):

    wall_in_range = dist[0] < constants.CONST_WALL_DIST - constants.CONST_WALL_OFFSET
    return wall_in_range



# check if we no longer "see" the object on the right
def is_right_wall_lost(dist): 

    return dist[5] < constants.CONST_INF_DIST


# check if we no longer "see" the object on the left
def is_left_wall_lost(dist): 
    
    return dist[0] < constants.CONST_INF_DIST


# check if there is more space on the right of the robot than on the left
def is_more_space_on_right(dist):

    values_on_right = dist[3] + dist[4] + dist[5]
    values_on_left  = dist[1] + dist[2] + dist[0]

    return (values_on_left > values_on_right) 


# check if the system is being unstuck
def is_being_unstuck(state):

    return (state is constants.STATE_STUCK_LEFT) or (state is constants.STATE_STUCK_RIGHT)


# check if it is more beneficial to unstuck by turning to the right
def should_unstuck_right(dist, wall_is_followed_left):

    no_preference_decision = (not wall_is_followed_left) and is_more_space_on_right(dist)
    return wall_is_followed_left or no_preference_decision

# check if robot is "bored"
def bored(wall_boredom_counter):
    return wall_boredom_counter >= constants.CONST_WALL_BORED_MAX

# check if we are handling boredom
def is_boredom_handled(state):
    return state == constants.STATE_BOREDOM_ROTATE or state == constants.STATE_BOREDOM_DRIVE


def main():


    try:
        #flashy to see if robot works
        comms.blinkyblink()
        odo1 = Odometry_Algorithm_1()
        odometry_state_1 = Odometry_State()
        # reset odometry for this robot run 
        comms.reset_odo()
	odo = [0,0]


	forward_counter = 0
	turn_counter = 0

	angle_saved = 0
	#begin control loop
        while True:
	    #TODO remove below line if does nto wortk
	    #comms.reset_odo()
	    #dist = comms.get_ir()

	    odo_new = comms.get_odo()
	    delta_odo = [odo_new[0] - odo[0], odo_new[1] - odo[1]]
	    odo = [delta_odo[0] + odo[0], delta_odo[1] + odo[1]]
		
	    odometry_state_1 = odo1.new_state(odometry_state_1, delta_odo)
            ds.push(odometry_state_1)
			
	    print("ODO #1 : THETA " + "{0:.0f}".format(math.degrees(odometry_state_1.theta)) + ")" )          
            

	    if(forward_counter >= 10):
		
		if abs(math.degrees(odometry_state_1.theta)) >=  90 :  #(turn_counter == 50):
			forward_counter = 0
			print ("DONE TURNING")
			comms.drive(constants.CONST_SPEED, constants.CONST_SPEED)
		else:
			comms.drive(constants.CONST_SPEED,-constants.CONST_SPEED)
			print ("TURNING")
	    else:
		comms.drive(constants.CONST_SPEED, constants.CONST_SPEED)
		forward_counter = forward_counter + 1

	    # do not attempt to instantly read sensors again
            time.sleep(constants.MEASUREMENT_PERIOD_S)

    except TypeError as e:
        comms.drive(0,0)
        raise(e)

# #####################
# Init & CLI gubbins...
# #####################

if __name__ == "__main__":
    # Ignore 1st member, which is the name 
    # the program was invoked with
    args = sys.argv[1:]

    # Read & Parse command line options
    try:
        optlist, args = getopt.getopt(args, 'hp:t:b:s:', 
                                      ['help','port=','server=','baud=','timeout='])
    except getopt.GetoptError:
        print("Invalid Option, correct usage:")
        print(helptext)
        sys.exit(2)

    # Our defaults, may be different 
    # from the ones built into the class'
    # __init__(self) constructor
    port = "/dev/ttyUSB0"
    timeout = 1
    baud = 9600

    server = "localhost"
        
    for opt, arg in optlist:
        if opt in ('-h', '--help'):
            print(namebadge)
            print(helptext)
            sys.exit(0)
        elif opt in ('-p', '--port'):
            # change serial port to use
            port = str(arg)
            
        elif opt in ('-t','--timeout'):
            # change blocking timeout for reading
            timeout = float(arg)
            
        elif opt in ('-b', '--baud'):
            # change baud rate
            baud = int(arg)
            
        elif opt in ('-s', '--server'):
            server = str(arg)
            print("Connecting to Redis server at " + str(server))
            
    # Initialise a serial class, or 
    try:
        comms = Comms(port, baud, timeout)
    except Exception as e:
        if wt.confirm("Can't initialise serial, exit?\n\n"+str(e)):
            sys.exit(1)
        raise(e)
            
    print(namebadge)
        
    ds = DataStore(host=server)
    
    try:
        main()
    except KeyboardInterrupt as e:
        comms.drive(0,0)
        ds.save()
        print("Stopping and Quitting...")
        raise e

else:
    # if *not* running as __main__
    # invoke the class with defaults
    comms = Comms()
    ds = DataStore()


