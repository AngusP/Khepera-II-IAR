#
#     - - - -  I A R  - - - -
#
#  s1311631         Angus Pearson
#  s1346981    Jevgenij Zubovskij
#

from __future__ import print_function
import serial # Documentation: http://pyserial.readthedocs.io/en/latest/index.html
import sys
from serial.tools import list_ports as list_ports

class Comms:

    port = serial.Serial()

    # Initialise the class, trying to open the Serial Port
    # with multiple levels of graceful failure.
    def __init__(self, port="/dev/ttyUSB0", baud=9600, timeout=0):
        
        self.port.baud = baud
        self.port.timeout = timeout

        try:
            self.port.port = port
            self.port.open()
        
        except serial.serialutil.SerialException as e1:
            print(e1)
            print("Trying alternate port...")
            
            try:
                self.port.port = "/dev/ttyUSB1"
                self.port.open()
                pass
            
            except serial.serialutil.SerialException as e2:
                print(e2)
                print("\n!!! CAN'T OPEN PORT !!!")
                
                # We can't open a port, enumerate them for the user
                print("\nAvailable Serial Ports:")
                for possible in list_ports.comports():
                    print(possible)
                print("")

            raise(e1)
                

    def __del__(self):
        if self.port.is_open:
            self.port.close()

    # directly control motor speeds
    def drive(self, lspeed, rspeed):
        pass

    def stop(self):
        self.drive(0,0)

    # Return odometry (wheel rotation) data
    def get_odo(self):
        pass

    def get_dist(self, sensor_no=None):
        pass

    # Reset the robot's wheel counts to 0
    def reset_odo(self):
        pass


