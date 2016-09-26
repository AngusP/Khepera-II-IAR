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
import time

class Comms:

    FORWARD = 0
    BACKWARD = 1
    LEFT = 2
    RIGHT = 3
    LEFT_FOLLOW = 4
    RIGHT_FOLLOW = 5
    BOREDOM_TURN_ON_SPOT = 6
    BOREDOM_RUN_AWAY = 7

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

    def _parse_sensor(self, string):
        data = string.strip('\n\r').split(",")
        data_ints = [int(d) for d in data[1:]]
        return data_ints

    def clear_port(self):
        self.port.reset_input_buffer()

    # directly control motor speeds
    def drive(self, lspeed, rspeed):
        cmd = "D," + str(int(lspeed)) + "," + str(int(rspeed)) + "\n"
        #print(cmd, end="")
        self.port.write(cmd)
        #print(self.port.readline(), end="")
        self.port.readline()

    # self-explanatory
    def stop(self):
        self.drive(0,0)

    # Return odometry (wheel rotation) data
    def get_odo(self):
        self.port.write("H\n")
        odo = self._parse_sensor(self.port.readline()) 
        return odo

    # Reset the robot's wheel counts to 0
    def reset_odo(self):
        self.port.write("G,0,0\n")

    # Return IR Distance measurements
    def get_ir(self, sensor_no=None):
        self.port.write("N\n")
        dist = self._parse_sensor(self.port.readline())

        if sensor_no is None:
            return dist
        else:
            return dist[sensor_no]


    # Return IR Ambient Light Measurements
    def get_ambient(self, sensor_no=None):
        self.port.write("O\n")
        amb = self._parse_sensor(self.port.readline()) 
        return amb

    # control status LEDs on the robot
    def led(self, led_num=None, state=1):
        if state not in [0,1]:
            state = 0

        if led_num is None or led_num == 0:
            self.port.write("L,0," + str(state) + "\n")
        else:
            self.port.write("L,1," + str(state) + "\n")


    def blinkyblink(self):
        self.led(0,1)
        self.led(1,0)
        time.sleep(0.1)
        self.led(0,0)
        self.led(1,1)
        time.sleep(0.1)
        self.led(0,1)
        self.led(1,0)
        time.sleep(0.1)
        self.led(0,0)
        self.led(1,1)
        time.sleep(0.1)
        self.led(0,1)
        self.led(1,0)
        time.sleep(0.1)
        self.led(0,0)
        self.led(1,1)
        time.sleep(0.1)
        self.led(0,0)
        self.led(1,0)
        time.sleep(0.1)
        self.clear_port()
