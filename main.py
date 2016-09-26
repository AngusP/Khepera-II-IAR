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
CONST_WALL_BORED_MAX = 50
CONST_BORED_TURN_MAX = 10

DIR_LEFT = 0
DIR_RIGHT = 1

def main():
    try:
        comms.blinkyblink()
        comms.drive(CONST_SPEED, CONST_SPEED)
        going = Comms.FORWARD
	wall_is_followed_left = False
	wall_is_followed_right = False
	
	wall_boredom_counter = 0
	wall_follow_previous_dir = DIR_LEFT

	boredom_turn_on_spot_counter = 0

        while True:
            dist = comms.get_ir()
	    print ("BOREDOM")
            print (wall_boredom_counter)

	    #record that we are still moving along the same wall (as we record cycles in the "mode")
	    if wall_is_followed_left or wall_is_followed_right:
	    	wall_boredom_counter +=1

	    ######################################################################################################################################
	    # IF RANDOM TURN ON SPOT
	    ###########################
	    
	    #TODO check if that is waht they wanted
	    if going == comms.BOREDOM_TURN_ON_SPOT and boredom_turn_on_spot_counter >= CONST_BORED_TURN_MAX:
	     	going = comms.BOREDOM_RUN_AWAY
		#RESET SYSTEM TO INITIAL STATE
		comms.drive(CONST_SPEED, CONST_SPEED)
	    elif going == comms.BOREDOM_TURN_ON_SPOT:
		boredom_turn_on_spot_counter += 1
		#TURN UNTIL LIMIT EXHAUSTED
		continue

	    ######################################################################################################################################
	    # IF STUCK
            #######################
 	    if (dist[3] + dist[2]) > CONST_WALL_DIST or (dist[1] + dist[4] > CONST_WALL_DIST*1.5):
                print("Wall!")
		# discontinue following the wall
		# print("STOPPING FOLLOWING") 
		
		if going is not comms.FORWARD and not (going == Comms.RIGHT_FOLLOW) and not (going == Comms.LEFT_FOLLOW) and not (going == comms.BOREDOM_TURN_ON_SPOT) and not (going == comms.BOREDOM_RUN_AWAY):
			continue
 
		
		wall_was_followed = wall_is_followed_left or wall_is_followed_right
		more_space_on_right = (dist[1] + dist[2] + dist[0]) > (dist[3] + dist[4] + dist[5])
		
                if more_space_on_right : #and wall_is_followed_left) or not (wall_is_followed_left and have_space_on_left):
			
                    going = Comms.RIGHT
                    comms.drive(CONST_SPEED,-CONST_SPEED)

                else:

                    going = Comms.LEFT
                    comms.drive(-CONST_SPEED, CONST_SPEED)
		

		# record the direction switch arising from the switch 
		if wall_is_followed_left:
			wall_follow_previous_dir = DIR_LEFT
		elif wall_is_followed_right:
			wall_follow_previous_dir = DIR_RIGHT

		wall_is_followed_left = False
		wall_is_followed_right = False
		#wall_boredom_counter = 0
		

	    ######################################################################################################################################
	    # IF (TOO) CLOSE TO WALL
	    #######################
	    elif not ((going == Comms.RIGHT) or (going == Comms.LEFT)) and  (dist[0] > CONST_WALL_DIST or dist[5] > CONST_WALL_DIST):
		
		#if wall is not being followed on the right
		if not wall_is_followed_right or (dist[0]  > dist[5]) :

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
			going = comms.RIGHT_FOLLOW

			
	    #######################################################################################################################################
	    # IF TOO FAR FROM WALL
	    #####################
            elif (dist[0] < CONST_WALL_DIST - CONST_WALL_OFFSET and wall_is_followed_left) or (dist[5] < CONST_WALL_DIST - CONST_WALL_OFFSET and wall_is_followed_right): 
		

		#if wall is not being followed on the right
		if not wall_is_followed_right and dist[0] + dist[1] + dist[2] > dist[3] + dist[4] + dist[5] :

			print("FOLLOWING / LEAVING WALL LEFT")
	
			wall_is_followed_left = True
			wall_is_followed_right = False
			comms.drive(CONST_SPEED * 0.5, CONST_SPEED)
			going = comms.LEFT_FOLLOW

		else:

			print("FOLLOWING / LEAVING WALL RIGHT")

			wall_is_followed_left = False
			wall_is_followed_right = True
			comms.drive(CONST_SPEED, CONST_SPEED * 0.5)
			going = comms.RIGHT_FOLLOW
			

	    ########################################################################################################################################
	    # IF NONE OF THE ABOVE
	    #####################
            else:
                if going is not Comms.FORWARD or wall_boredom_counter >= CONST_WALL_BORED_MAX:
			
  
		   
		    
		    #stuff was on the right, so should stay to right wall
		    if(going is comms.LEFT):
			wall_is_followed_right = True
			if wall_follow_previous_dir == DIR_LEFT:
				wall_boredom_counter = 0
				
		    #otherwise follow stuff on the left
		    elif(going is comms.RIGHT):
			wall_is_followed_left = True
			if wall_follow_previous_dir == DIR_RIGHT:
				wall_boredom_counter = 0
			
		    #check if the boredom counter was exceeded
                    if wall_boredom_counter >= CONST_WALL_BORED_MAX:
			
			wall_boredom_counter = 0
			boredom_turn_on_spot_counter = 0
		

			wall_is_followed_left = False
			wall_is_followed_right = False
			#go off in a BOREDOM_TURN_ON_SPOT direction until we encounter a wall TODO 
	   		
			#turn for a number of cycles before moving away from the wall
			going = comms.BOREDOM_TURN_ON_SPOT
			comms.drive(CONST_SPEED, -CONST_SPEED)

		    #otherwise just drive straight
		    else:
			
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


