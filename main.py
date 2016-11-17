#!/usr/bin/env python

#
#     - - - -  I A R  - - - -
#
#  s1311631         Angus Pearson
#  s1346981    Jevgenij Zubovskij
#

from comms 			import Comms

from odometry_algorithm 	import Odometry_Algorithm
from odometry_state 		import Odometry_State

from navigation_state 		import Navigation_State
from navigation_algorithm 	import Navigation_Algorithm

from pathing_algorithm    	import Pathing_Algorithm
from pathing_state 	  	import Pathing_State

from mapping import Particles, Mapping

import constants
import sys
import cv2
import getopt      # CLI Option Parsing
import whiptail    # Simplest kinda-GUI thing
import time
import math
import matplotlib.pyplot as plt
from data import DataStore
from data import GridManager

namebadge = " -- IAR C&C -- "
helptext = str(sys.argv[0]) + ' -p <serial port> -b <baud rate> -t <timeout> -s <server hostname>'

wt = whiptail.Whiptail(title=namebadge)



def main():

    cv2.namedWindow("window")
    try:
        #flashy to see if robot works
        comms.blinkyblink()
        
        odo = Odometry_Algorithm()
        odo_state = Odometry_State()
	nav_state = Navigation_State()
	nav = Navigation_Algorithm()
	pathing_state = Pathing_State()

	#TODO sync with angus' granularity in mm
    	pathing = Pathing_Algorithm(20, ds)

        # varaibles to not resend speeds during wall following
        speed_l = 0
        speed_r = 0
    
        # reset odometry for this robot run 
        comms.reset_odo()
	 
        t0 = time.time()
        prior_odo = None

	#begin control loop
        while True:

	    #get new sensor readings
            prior_odo = odo_state
	    odo_state = odo.new_state(odo_state, comms.get_odo())
  	    nav_state.dist = comms.get_ir()

	    #check reactive controls first
	    nav_state = nav.new_state(nav_state ,pathing_state, comms, ds)
	    #then check if pathing algorithm applies        
	    nav_state = pathing.new_state(nav_state, odo_state, pathing_state, comms)

            # Push deltas to particle filter
            t1 = time.time()
            dt = t1 - t0
            t0 = t1
            
            delta_s = m._euclidean((prior_odo.x, prior_odo.y), (odo_state.x, odo_state.y))
            dtheta = prior_odo.theta - odo_state.theta
            #              dt, ds,      dtheta, sensor readings
            pf.push_params(dt, delta_s, dtheta, nav_state.dist)
            predict_state = pf()
            print(predict_state)

	    #if we are done break the control loop, stop the robot and exit
	    if pathing_state.done:
		  #make a repeated run	
		  pathing_state.algorithm_activated = True
		  pathing_state.done = False
		  pathing.replan_sequence(pathing_state)
		  
		  #print "DONE REPLAN"
			

	    #only send stuff over serial if new values
	    if not( speed_l == nav_state.speed_l and speed_r == nav_state.speed_r):
	    	comms.drive(nav_state.speed_l, nav_state.speed_r)
	    #update the speeds
            speed_l = nav_state.speed_l
            speed_r = nav_state.speed_r
            ds.push(odo_state, nav_state.dist)

	    
	    # wait for new sensor round 
	    key_pressed = cv2.waitKey(constants.MEASUREMENT_PERIOD_MS) & 0xFF
	    #found a new food source 
	    if key_pressed  == ord(' '):
			# if SPACE is pressed
   			print("Detected food")
			pathing.drive_over_food(pathing_state, comms)

	    #collected food from current food_source
	    elif key_pressed  == ord('\r'):
			# if ENTER pressed
			#collect the food we spiralled around before
			pathing.collect_food(pathing_state, comms)
 	   
		
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

    server = "/tmp/redis.sock"
        
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
    m = Mapping(ds=ds)
    pf = Particles(mapper=m)
    
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
    ds = DataStore()
