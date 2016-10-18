#!/usr/bin/env python

#
#     - - - -  I A R  - - - -
#
#  s1311631         Angus Pearson
#  s1346981    Jevgenij Zubovskij
#

from __future__ 		import print_function
from comms 			import Comms

from odometry_algorithm 	import Odometry_Algorithm
from odometry_state 		import Odometry_State

from navigation_state 		import Navigation_State
from navigation_algorithm 	import Navigation_Algorithm

from bug_algorithm 		import Bug_Algorithm
from bug_state 			import Bug_State



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



def main():


    try:
        #flashy to see if robot works
        comms.blinkyblink()
        
        odo = Odometry_Algorithm()
        odo_state = Odometry_State()
	nav_state = Navigation_State()
	nav = Navigation_Algorithm()
	bug = Bug_Algorithm()
	bug_state = Bug_State()


        # varaibles to not resend speeds during wall following
        speed_l = 0
        speed_r = 0

        # List of dict for Redis -> ROS goal line render
        mline_hm = [
            {'x': bug_state.m_line_start[0],
             'y': bug_state.m_line_start[1]},
            {'x': bug_state.m_line_end[0],
             'y': bug_state.m_line_end[1]}
            ]
    
        # reset odometry for this robot run 
        comms.reset_odo()
	 

	#begin control loop
        while True:
          

	    odo_state = odo.new_state(odo_state, comms.get_odo())
  	    nav_state.dist = comms.get_ir()

	    #check reactive first, then bug
	    nav_state = nav.new_state(nav_state, odo_state, bug_state, comms)
	    #if have free movement, use the bug algorithm
	    if bug_state.algorithm_activated and bug_state.in_control == True:
		nav_state = bug.new_state(nav_state, odo_state, bug_state)
		if bug_state.done:
			print("DONE")
			comms.drive(0, 0)
			comms.blinkyblink()
			break
	
	    #only send stuff over serial if new values
	    if not( speed_l == nav_state.speed_l and speed_r == nav_state.speed_r):
	    	comms.drive(nav_state.speed_l, nav_state.speed_r)

            speed_l = nav_state.speed_l
            speed_r = nav_state.speed_r
	    
            ds.push(odo_state, nav_state.dist)

            if (mline_hm[1]['x'] != bug_state.m_line_end[0] or
                mline_hm[1]['y'] != bug_state.m_line_end[1]):
                # Update and push
                mline_hm[1]['x'] = bug_state.m_line_end[0]
                mline_hm[1]['y'] = bug_state.m_line_end[1]
                ds.push_goal(mline_hm)
            
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
