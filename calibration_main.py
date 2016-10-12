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

	drive_counter = 0
	saved_angle = 0

	speed = 2
	#begin control loop
        while True:
	    drive_counter +=1;
	    odo_new = comms.get_odo()
	    delta_odo = [odo_new[0] - odo[0], odo_new[1] - odo[1]]
	    odo = [delta_odo[0] + odo[0], delta_odo[1] + odo[1]]
		
	    odometry_state_1 = odo1.new_state(odometry_state_1, delta_odo)
            ds.push(odometry_state_1)

	    if drive_counter > 15:
		    if drive_counter < 40:
		    	comms.drive(5,5)
			drive_counter +=1
			saved_angle = odometry_state_1.theta
		    else:
			drive_counter = 0
		
	    else:
		if drive_counter % 5 == 0:
			speed += 1	
	        comms.drive(speed,-2)	


	    #print("ODO #1 : THETA " + "{0:.0f}".format(math.degrees(odometry_state_1.theta)) + ")" )          
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


