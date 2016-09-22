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
    def __init__(self, port="/dev/ttyUSB0", baud=9600, timeout=1):
        
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
        self.clear_port()
                

    def __del__(self):
        if self.port.is_open:
            self.port.close()

    def clear_port(self):
        self.port.reset_input_buffer()

    # directly control motor speeds
    def drive(self, lspeed, rspeed):
        cmd = "D," + str(int(lspeed)) + "," + str(int(rspeed)) + "\n"
        print(cmd, end="")
        self.port.write(cmd);
        print(self.port.readline(), end="")

    def stop(self):
        self.drive(0,0)

    # Return odometry (wheel rotation) data
    def get_odo(self):
        self.port.write("H\n")
        print("Odometry:")
        print(self.port.readline(), end="")

    # Reset the robot's wheel counts to 0
    def reset_odo(self):
        self.port.write("G,0,0\n")

    # Return IR Distance measurements
    def get_ir(self, sensor_no=None):
        self.port.write("N\n")
        print("IR Distances:")
        print(self.port.readline(), end="")

    # Return IR Ambient Light Measurements
    def get_ambient(self, sensor_no=None):
        self.port.write("O\n")
        print("IR Ambient Light:")
        print(self.port.readline(), end="")

    def set_led(self, state=1, lednum=None):
        if state not in [0,1]:
            state = 0

        if led_num is None or lednum == 0:
            self.port.write("L,0," + str(state) + "\n")
        else:
            self.port.write("L,1," + str(state) + "\n")
            


