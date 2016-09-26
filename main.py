#!/usr/bin/env python

#
#     - - - -  I A R  - - - -
#
#  s1311631         Angus Pearson
#  s1346981    Jevgenij Zubovskij
#

from __future__ import print_function
from comms import Comms
import sys
import getopt      # CLI Option Parsing
import whiptail    # Simplest kinda-GUI thing
import time
import math
import matplotlib.pyplot as plt

namebadge = " -- IAR C&C -- "
helptext = str(sys.argv[0]) + ' -p <serial port> -b <baud rate> -t <timeout>'

wt = whiptail.Whiptail(title=namebadge)

CONST_SPEED = 8
CONST_WALL_DIST = 200
CONST_WALL_OFFSET = 40
CONST_WALL_BORED_MAX = 2000

def main():
    try:
        comms.blinkyblink()
        comms.drive(CONST_SPEED, CONST_SPEED)
        going = Comms.FORWARD
	wall_is_followed_left = False
	wall_is_followed_right = False
	wall_is_followed_in_same_direction = 0 # TODO 

        while True:
            dist = comms.get_ir()
	    ######################################################################################################################################
	    # IF it is stuck
            if (dist[1] + dist[2]) > CONST_WALL_DIST*1.5 or (dist[3] + dist[4]) > CONST_WALL_DIST*1.5:
                print("Wall!")
		# discontinue following the wall
		# print("STOPPING FOLLOWING")

		
		if going is not comms.FORWARD and not (going == Comms.RIGHT_FOLLOW) and not (going == Comms.LEFT_FOLLOW):
			continue
 
		
		wall_was_followed = wall_is_followed_left or wall_is_followed_right
		more_space_on_right = (dist[1] + dist[2]) > (dist[3] + dist[4])
		have_space_on_left = dist[5] > CONST_WALL_DIST
		
                if (more_space_on_right and not wall_was_followed) or not (wall_is_followed_left and have_space_on_left):
			
                    going = Comms.RIGHT
                    comms.drive(CONST_SPEED,-CONST_SPEED)

                else:

                    going = Comms.LEFT
                    comms.drive(-CONST_SPEED, CONST_SPEED)

		wall_is_followed_left = False
		wall_is_followed_right = False
		#wall_is_followed_in_same_direction = 0
		

	    ######################################################################################################################################
	    #IF too close to a wall (or found one)
	    elif not ((going == Comms.RIGHT) or (going == Comms.LEFT)) and  (dist[0] > CONST_WALL_DIST or dist[5] > CONST_WALL_DIST):
		
		#if wall is not being followed on the right
		if not wall_is_followed_right:

			print("FOLLOWING / APPROACHING WALL LEFT")
	
			wall_is_followed_left = True
			wall_is_followed_right = False
			comms.drive(CONST_SPEED, CONST_SPEED*0.5)
			going = comms.LEFT_FOLLOW

		else:

			print("FOLLOWING / APPROACHING WALL RIGHT")

			wall_is_followed_left = False
			wall_is_followed_right = True
			comms.drive(CONST_SPEED * 0.5, CONST_SPEED)
			going = comms.LEFT_FOLLOW
			
	    #######################################################################################################################################
	    #IF too far from a wall
            elif (dist[0] < CONST_WALL_DIST - CONST_WALL_OFFSET and wall_is_followed_left) or (dist[5] < CONST_WALL_DIST - CONST_WALL_OFFSET and wall_is_followed_right): 
		

		#if wall is not being followed on the right
		if not wall_is_followed_right:

			print("FOLLOWING / LEAVING WALL LEFT")
	
			wall_is_followed_left = True
			wall_is_followed_right = False
			comms.drive(CONST_SPEED * 0.5, CONST_SPEED)
			going = comms.RIGHT_FOLLOW

		else:

			print("FOLLOWING / LEAVING WALL RIGHT")

			wall_is_followed_left = False
			wall_is_followed_right = True
			comms.drive(CONST_SPEED, CONST_SPEED * 0.5)
			going = comms.RIGHT_FOLLOW
			

		print(dist[0])
		comms.drive(CONST_SPEED*0.5, CONST_SPEED)
		going = comms.RIGHT_FOLLOW

	    ########################################################################################################################################
	    #IF can drive straight
            else:
                if going is not Comms.FORWARD:

                    going = Comms.FORWARD
                    comms.drive(CONST_SPEED,CONST_SPEED)
                
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


