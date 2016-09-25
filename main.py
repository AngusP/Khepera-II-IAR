#!/usr/bin/env python

#
#     - - - -  I A R  - - - -
#
#  s1311631         Angus Pearson
#  s1346981    Jevgenij Zubovskij
#

from __future__ import print_function
from comms import Comms
from pid_control import PID_control
import sys
import getopt      # CLI Option Parsing
import whiptail    # Simplest kinda-GUI thing
import time
import math
import matplotlib.pyplot as plt

namebadge = " -- IAR C&C -- "
helptext = str(sys.argv[0]) + ' -p <serial port> -b <baud rate> -t <timeout>'

wt = whiptail.Whiptail(title=namebadge)
pid = PID_control()

SPEED_CONST = 10

def main():
    try:
        comms.blinkyblink()
        comms.drive(5,5)
	

	speed = [0,0]
	
	turn_stuck = "none"
	
	wall_reached = False

        while True:	
	    
	    #get sensor readings
	    dists = comms.get_ir()
	    err = pid.pid_distance(dists)


	    if turn_stuck == "none":
		    speed = [10,10]
		    if err[0] > 0 or err[1] > 0:
			speed[1] = speed[1] * 0.5
			wall_reached = True

		    elif err[0] < 0 and wall_reached:
			speed[1] = speed[1] * 1.2
			wall_reached = False
		    

		    #if needs to turn to get unstuck
		    if err[2] > 0 or err[3] > 0:
		        print("Wall!")
		        if (err[0] > err[5] or err[1] > err[4]):
			    turn_stuck = "right"
			    speed = [5,-5]
		        else:
			    turn_stuck = "left"
			    speed = [-5, 5]			
		
		   
	    else:
		if err[2] <= 0 and err[3] <= 0:
			turn_stuck = "none"
	    
	    #normalize the speed vector and drive
  	    speed_vector_length = math.sqrt( speed[0] ** 2 + speed[1] ** 2)
	    speed = [speed[0] * SPEED_CONST / speed_vector_length, speed[1]* SPEED_CONST / speed_vector_length]
	    comms.drive(speed[0], speed[1])
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


