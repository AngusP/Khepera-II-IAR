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

        #initialise variables
        wall_is_followed_left = False
        wall_is_followed_right = False
        wall_boredom_counter = 0
        boredom_turn_on_spot_counter = 0

        #set odometry state to 0
        #TODO check whihc one works better
        
        odo1 = Odometry_Algorithm_1()
        odo2 = Odometry_Algorithm_2()

        odometry_state_1 = Odometry_State()
        odometry_state_2 = Odometry_State()
        
        #drive forward
        system_state = constants.STATE_DRIVE_FORWARD
        comms.drive(constants.CONST_SPEED, constants.CONST_SPEED)

        # varaibles to not resend speeds during wall following
        speed_l = constants.CONST_SPEED
        speed_r = constants.CONST_SPEED
    
        # reset odometry for this robot run 
        comms.reset_odo()

	odo = [0,0]
	#begin control loop
        while True:
	    #TODO remove below line if does nto wortk
	    #comms.reset_odo()
	    dist = comms.get_ir()

	    odo_new = comms.get_odo()
	    delta_odo = odo_new - odo
	    odo = odo + delta_odo
		
	    odometry_state_1 = odo1.new_state(odometry_state_1, delta_odo)
	    odometry_state_2 = odo2.new_state(odometry_state_2, delta_odo)
			
	    #print("ODO #1 : X (" + "{0:.2f}".format(odometry_state_1.x)+ ", Y" + "{0:.2f}".format(odometry_state_1.y) + ", THETA " + "{0:.0f}".format(odometry_state_1.theta) + ")" )
	    #print("ODO #2 : X (" + "{0:.2f}".format(odometry_state_2.x)+ ", Y" + "{0:.2f}".format(odometry_state_2.x) + ", THETA " + "{0:.0f}".format(odometry_state_2.theta) + ")" )
            
            print("ODO new " + str(odo[0]) + " , " + str(odo[1]))
            
            ########################
            #HANDLE BOREDOM COUNTER 
            ########################
            
            #record that we are still moving along a wall and not "exploring"
            if wall_is_followed_left or wall_is_followed_right:
                wall_boredom_counter +=1

            
            #check if robot is "bored" and it is not being handled
            if bored(wall_boredom_counter) and not is_boredom_handled(system_state):

                print("bored...")

                #turn away from the wall that was last followed
                if wall_is_followed_left:
                    comms.drive(constants.CONST_SPEED, -constants.CONST_SPEED)
                elif wall_is_followed_right:
                    comms.drive(-constants.CONST_SPEED, constants.CONST_SPEED)
                
                # reset state
                boredom_turn_on_spot_counter = 0
                wall_boredom_counter = 0
                wall_is_followed_left = False
                wall_is_followed_right = False
                        
                # set state
                system_state = constants.STATE_BOREDOM_ROTATE
                #go to next iteration of the loop
                continue


            # if we are rotating on the spot                  
            if system_state == constants.STATE_BOREDOM_ROTATE:
                #check if we are done rotating
                if boredom_turn_on_spot_counter >= constants.CONST_BORED_TURN_MAX:
                    #different from normal forward driving as we try to get very far from the wall
                    system_state = constants.STATE_BOREDOM_DRIVE
                    comms.drive(constants.CONST_SPEED, constants.CONST_SPEED)

                # otherwise, continue rotationg for this loop iteration
                else:
                    boredom_turn_on_spot_counter += 1
                    #go to next iteration of the loop
                    continue

            ############################
            # IF STUCK
            ############################

            if is_stuck(dist):
                
                print("stuck")

                # do not interrupt if already handle
                if is_being_unstuck(system_state):
                        continue

                # determine direction of where better to turn to unstuck
                if should_unstuck_right(dist, wall_is_followed_left):

                    system_state = constants.STATE_STUCK_RIGHT
                    comms.drive(constants.CONST_SPEED,-constants.CONST_SPEED)

                else:

                    system_state = constants.STATE_STUCK_LEFT
                    comms.drive(-constants.CONST_SPEED, constants.CONST_SPEED)
                

                # stop following the wall (as it could ahve potentially led to being stuck)
                wall_is_followed_left = False
                wall_is_followed_right = False
                boredom_counter = 0


            # if robot not stuck and we are driving away from a "boring" wall, continue doing so
            elif system_state == constants.STATE_BOREDOM_DRIVE:
                continue

            #####################
            ##WALL FOLLOWING LEFT
            #####################

            elif (wall_is_followed_left or should_follow_left_wall(dist)) and not (is_left_wall_lost(dist)):
                
                print("following left")

                # set state accordingly 
                wall_is_followed_left = True
                wall_is_followed_right = False
                system_state = constants.STATE_LEFT_FOLLOW

                # keep the distance within the threshold range
                if too_close_to_left(dist):

                    speed_l = constants.CONST_SPEED
                    speed_r = constants.CONST_SPEED * constants.CONST_TURN_PROPORTION
                    comms.drive(speed_l, speed_r)

                elif is_away_from_left(dist):

                    speed_l = constants.CONST_SPEED * constants.CONST_TURN_PROPORTION
                    speed_r = constants.CONST_SPEED
                    comms.drive(speed_l, speed_r)

                else: 

                    speed_l = constants.CONST_SPEED
                    speed_r = constants.CONST_SPEED
                    comms.drive(constants.CONST_SPEED, constants.CONST_SPEED)

            #####################
            ##WALL FOLLOWING RIGHT
            #####################                       
            elif (wall_is_followed_right or should_follow_right_wall(dist)) and not (is_right_wall_lost(dist)):
                
                print("following right")

                # set state accordingly
                wall_is_followed_left = False
                wall_is_followed_right = True
                system_state = constants.STATE_RIGHT_FOLLOW

                # keep the distance within the threshold range 
                if too_close_to_right(dist):

                    speed_l = constants.CONST_SPEED * constants.CONST_TURN_PROPORTION
                    speed_r = constants.CONST_SPEED
                    comms.drive(speed_l, speed_r)

                elif is_away_from_right(dist):

                    speed_l = constants.CONST_SPEED
                    speed_r = constants.CONST_SPEED * constants.CONST_TURN_PROPORTION
                    comms.drive(speed_l, speed_r)

                else: 

                    speed_l = constants.CONST_SPEED
                    speed_r = constants.CONST_SPEED
                    comms.drive(speed_l, speed_r)


            ######################
            # IF NONE OF THE ABOVE
            #####################
            else:

                print("driving...")

                # otherwise just drive forward
                if system_state is not constants.STATE_DRIVE_FORWARD: 

                    # reset variables as not doing anything
                    wall_is_followed_left = False
                    wall_is_followed_right = False
                    wall_boredom_counter = 0

                    # set state accordingly
                    system_state = constants.STATE_DRIVE_FORWARD
                    comms.drive(constants.CONST_SPEED,constants.CONST_SPEED)

	    
	    # do not attempt to instantly read sensors again
            time.sleep(constants.MEASUREMENT_PERIOD_S)

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


