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


namebadge = " -- IAR C&C -- "
helptext = str(sys.argv[0]) + ' -p <serial port> -b <baud rate> -t <timeout>'

wt = whiptail.Whiptail(title=namebadge)

# #####################
# Init & CLI gubbins...
# #####################
if __name__ == "__main__":
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
    timeout = 0
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
    
    try:
        comms = Comms(port, baud, timeout)
    except Exception as e:
        if wt.confirm("Can't initialise serial, exit?\n\n"+str(e)):
            sys.exit(1)

else:
    # if *not* running as __main__
    # invoke the class with defaults
    comms = Comms()

