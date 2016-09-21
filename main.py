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
import getopt

helptext = str(sys.argv[0]) + ' -p <serial port> -b <baud rate> -t <timeout>'

# #####################
# Init & CLI gubbins...
# #####################
if __name__ == "__main__":
    args = sys.argv[1:]

    # Read & Parse command line options
    try:
        optlist, args = getopt.getopt(args, 'hp:t:b:')
    except getopt.GetoptError:
        print("Invalid Option, correct usage:")
        print(helptext)
        sys.exit(2)

    port = "/dev/ttyUSB0"
    timeout = 1.0
    baud = 9600
        
    for opt, arg in optlist:
        if opt == '-h':
            print(helptext)
        
        elif opt == '-p':
            # change serial port to use
            port = str(arg)
            
        elif opt == '-t':
            # change blocking timeout for reading
            timeout = float(arg)

        elif opt == '-b':
            # change baud rate
            baud = int(arg)
    
    comms = Comms(port, baud, timeout)

else:
    # if *not* running as __main__:
    comms = Comms()

            

