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
CONST_WALL_OFFSET = 100
CONST_WALL_45 = CONST_WALL_DIST *math.cos(math.pi / 4)
CONST_WALL_75 = CONST_WALL_DIST *math.cos(1.2)

def main():
    try:
        comms.blinkyblink()
        comms.drive(CONST_SPEED, CONST_SPEED)
        going = Comms.FORWARD
	wall_found = False

        while True:
            dist = comms.get_ir()
	    # IF it is stuck
            if (dist[1] + dist[2]) > CONST_WALL_DIST*2:
                print("Wall!")
		# discontinue following the wall
		wall_found = False
                if (dist[1] + dist[2]) > (dist[3] + dist[4]):

                    going = Comms.RIGHT
                    comms.drive(CONST_SPEED,-CONST_SPEED)

                else:

                    going = Comms.LEFT
                    comms.drive(-CONST_SPEED,CONST_SPEED)

	    #IF too close to a wall (or found one)
	    elif dist[0] > CONST_WALL_DIST and not (going == Comms.RIGHT) and not (going == Comms.LEFT):
		print("WALL FOUND, FOLLOWING")
		print(dist[0])
		wall_found = True
		comms.drive(CONST_SPEED, CONST_SPEED*0.5)
		going = comms.LEFT_FOLLOW

	    #IF too far from a wall
            elif dist[0] < CONST_WALL_DIST - 20 and wall_found: # minus (-) because sensor value, not distance
		
		print(dist[0])
		comms.drive(CONST_SPEED*0.5, CONST_SPEED)
		going = comms.RIGHT_FOLLOW

	    #IF can drive straight
            else:
                if going is not Comms.FORWARD:
		    print(dist)
		    print("WALL LOST")
		    #wall_found = False
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


