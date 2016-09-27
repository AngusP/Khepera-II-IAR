#!/usr/bin/env python

#
#     - - - -  I A R  - - - -
#
#  s1311631         Angus Pearson
#  s1346981    Jevgenij Zubovskij
#

from __future__ import print_function
from comms import Comms


import constants
import sys
import getopt      # CLI Option Parsing
import whiptail    # Simplest kinda-GUI thing
import time
import math
import matplotlib.pyplot as plt

namebadge = " -- IAR C&C -- "
helptext = str(sys.argv[0]) + ' -p <serial port> -b <baud rate> -t <timeout>'

wt = whiptail.Whiptail(title=namebadge)



def is_away_from_right(dist):

	wall_in_range = dist[5] < constants.CONST_WALL_DIST - constants.CONST_WALL_OFFSET
	return wall_in_range  

def is_away_from_left(dist):

	wall_in_range = dist[0] < constants.CONST_WALL_DIST - constants.CONST_WALL_OFFSET
	return wall_in_range

def is_stuck(dist):

	stuck_cone_left  = dist[3] + dist[2] > constants.CONST_WALL_DIST*1.5 
	stuck_cone_right = dist[1] + dist[4] > constants.CONST_WALL_DIST*1.5
	stuck_cone_front = dist[3] + dist[2] > constants.CONST_WALL_DIST*1.5
	return stuck_cone_left or stuck_cone_front or stuck_cone_right

def should_follow_left_wall(dist):
	return dist[0] > constants.CONST_WALL_DIST and dist[0] > dist[5]

def should_follow_right_wall(dist):
	return dist[5] > constants.CONST_WALL_DIST and dist[5] > dist[0]

def too_close_to_left(dist):
	return dist[0] > constants.CONST_WALL_DIST

def too_close_to_right(dist):
	return dist[5] > constants.CONST_WALL_DIST

def is_right_wall_lost(dist): 
	return dist[5] < constants.CONST_INF_DIST

def is_left_wall_lost(dist): 
	return dist[0] < constants.CONST_INF_DIST

def is_more_space_on_right(dist):
	values_on_right = dist[3] + dist[4] + dist[5]
	values_on_left = dist[1] + dist[2] + dist[0]
	return values_on_left > values_on_right

def is_being_unstuck(system_state):
	return (system_state is constants.STATE_STUCK_LEFT) or (system_state is constants.STATE_STUCK_RIGHT)

def should_unstuck_right(dist, wall_is_followed_left):
	no_preference_decision = not wall_is_followed_left and is_more_space_on_right(dist)
	return wall_is_followed_left or no_preference_decision



def main():

    system_state = constants.STATE_FORWARD
    try:
        comms.blinkyblink()
        comms.drive(constants.CONST_SPEED, constants.CONST_SPEED)

	wall_is_followed_left = False
	wall_is_followed_right = False
	
	wall_boredom_counter = 0
	wall_follow_previous_dir = constants.DIR_LEFT
	boredom_turn_on_spot_counter = 0

        while True:
            dist = comms.get_ir()

	    ########################
	    #HANDLE BOREDOM COUNTER 
	    ########################
	    #record that we are still moving along the same wall (as we record cycles in the "mode")
	    if wall_is_followed_left or wall_is_followed_right:
	    	wall_boredom_counter +=1

	    
            #check if the boredom counter was exceeded
            if wall_boredom_counter >= constants.CONST_WALL_BORED_MAX:
			

		#turn away from the wall
		if wall_is_followed_left:
			comms.drive(constants.CONST_SPEED, -constants.CONST_SPEED)
		elif wall_is_followed_right:
			comms.drive(-constants.CONST_SPEED, constants.CONST_SPEED)
		
		#reset state
		wall_boredom_counter = 0
		boredom_turn_on_spot_counter = 0
		wall_is_followed_left = False
		wall_is_followed_right = False
		wall_follow_previous_dir = constants.DIR_LEFT
	   		
		#turn for a number of cycles before moving away from the wall
		system_state = constants.STATE_BOREDOM_ROTATE
			              
	    if system_state == constants.STATE_BOREDOM_ROTATE and boredom_turn_on_spot_counter >= constants.CONST_BORED_TURN_MAX:
		#different from normal forward driving as we try to get very far from the wall
	     	system_state = constants.STATE_BOREDOM_DRIVE
		comms.drive(constants.CONST_SPEED, constants.CONST_SPEED)

	    elif system_state == constants.STATE_BOREDOM_ROTATE:
		boredom_turn_on_spot_counter += 1
		#TURN UNTIL LIMIT EXHAUSTED
		continue

	    ############################
	    # IF STUCK
            ############################
 	    if is_stuck(dist):
		
		if is_being_unstuck(system_state):
			continue

		#if wall is not being followed on the right
		if should_unstuck_right(dist, wall_is_followed_left):

                    system_state = constants.STATE_STUCK_RIGHT
                    comms.drive(constants.CONST_SPEED,-constants.CONST_SPEED)

                else:

                    system_state = constants.STATE_STUCK_LEFT
                    comms.drive(-constants.CONST_SPEED, constants.CONST_SPEED)
		

		# record the direction switch arising from the switch 
		if wall_is_followed_left:
			wall_follow_previous_dir = constants.DIR_LEFT
		elif wall_is_followed_right:
			wall_follow_previous_dir = constants.DIR_RIGHT

		#stop following the wall
		wall_is_followed_left = False
		wall_is_followed_right = False


	    #only do something if can no longer go forward, else continue "running away"
	    elif system_state == constants.STATE_BOREDOM_DRIVE:
		continue

	    #####################
	    ##WALL FOLLOWING LEFT
	    #####################
            elif (wall_is_followed_left or should_follow_left_wall(dist)) and not (is_left_wall_lost(dist)):

	    	wall_is_followed_left = True
		wall_is_followed_right = False
		system_state = constants.STATE_LEFT_FOLLOW

		if too_close_to_left(dist):
			comms.drive(constants.CONST_SPEED, constants.CONST_SPEED*0.5)

                elif is_away_from_left(dist):
			comms.drive(constants.CONST_SPEED * constants.CONST_TURN_PROPORTION, constants.CONST_SPEED)
		else:
		     	comms.drive(constants.CONST_SPEED, constants.CONST_SPEED)

	    #####################
	    ##WALL FOLLOWING RIGHT
	    #####################			
            elif (wall_is_followed_right or should_follow_right_wall(dist)) and not (is_right_wall_lost(dist)):

	    	wall_is_followed_left = False
		wall_is_followed_right = True
		system_state = constants.STATE_RIGHT_FOLLOW

		if too_close_to_right(dist):
		    comms.drive(constants.CONST_SPEED*constants.CONST_TURN_PROPORTION, constants.CONST_SPEED)

                elif is_away_from_right(dist):
		    comms.drive(constants.CONST_SPEED, constants.CONST_SPEED*constants.CONST_TURN_PROPORTION)
		else:
		    comms.drive(constants.CONST_SPEED, constants.CONST_SPEED)


	    ######################
	    # IF NONE OF THE ABOVE
	    #####################
            else:
                if system_state is not constants.STATE_FORWARD: 
		    wall_is_followed_left = False
		    wall_is_followed_right = False
                    system_state = constants.STATE_FORWARD
                    comms.drive(constants.CONST_SPEED,constants.CONST_SPEED)
                
            time.sleep(0.02)
    except Exception as e:
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
        optlist, args = getopt.getopt(args, 'hp:t:b:', ['help'])
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
        
    for opt, arg in optlist:
        if opt in ('-h', '--help'):
            print(namebadge)
            print(helptext)
            sys.exit(0)
        
        elif opt == '-p':
            # change serial port to use
            port = str(arg)
            
        elif opt == '-t':
            # change blocking timeout for reading
            timeout = float(arg)

        elif opt == '-b':
            # change baud rate
            baud = int(arg)
    
    # Initialise a serial class, or 
    try:
        comms = Comms(port, baud, timeout)
    except Exception as e:
        if wt.confirm("Can't initialise serial, exit?\n\n"+str(e)):
            sys.exit(1)
        raise(e)

    print(namebadge)

    try:
        main()
    except KeyboardInterrupt as e:
        comms.drive(0,0)
        print("Stopping and Quitting...")
        raise e

else:
    # if *not* running as __main__
    # invoke the class with defaults
    comms = Comms()


