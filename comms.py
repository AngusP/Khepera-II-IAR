#
#     - - - -  I A R  - - - -
#
#  s1311631         Angus Pearson
#  s1346981    Jevgenij Zubovskij
#

from __future__ import print_function
import serial
from serial.tools import list_ports as list_ports

class Comms:

    port = serial.Serial()

    def __init__(self, port="/dev/ttyUSB0", baud=9600, timeout=1.0):
        
        self.port.baud = baud
        self.port.timeout = timeout

        try:
            self.port.port = port
            self.port.open()
        
        except serial.serialutil.SerialException as e:
            print(e)
            print("Trying alternate port...")
            
            try:
                self.port.port = "/dev/ttyUSB1"
                self.port.open()
            
            except serial.serialutil.SerialException as e:
                print(e)
                print("\n!!! CAN'T OPEN PORT !!!")
                
                # We can't open a port, enumerate them for the user
                print("\nAvailable Serial Ports:")
                for possible in list_ports.comports():
                    print(possible)
                print("")

                #raise(e)
                

    def __del__(self):
        if self.port.is_open:
            self.port.close()

    def drive(self, lspeed, rspeed):
        pass
        
