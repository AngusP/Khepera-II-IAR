#!/usr/bin/env/python

#
#     - - - -  I A R  - - - -
#
#  s1311631         Angus Pearson
#  s1346981    Jevgenij Zubovskij
#

from __future__ import print_function
from data import DataStore
import utils
import sys
import getopt
import math


testpose = {
    'r0': '92.0',
    'r1': '52.0',
    'r2': '36.0',
    'r3': '28.0',
    'r4': '48.0',
    'r5': '52.0',
    'r6': '56.0',
    'r7': '28.0',
    't': '1478358236.226565',
    'theta': '-0.5446428571428569',
    'x': '359.79165663643647',
    'y': '-279.4477174535753'
}



class Point(int):
    
    def __int__(self, x, y):
        self.x = x
        self.y = y


class Mapping(object):

    def __init__(self, host='localhost'):
        
        self.ds = DataStore(host=server)

        # Angles of the sensor from the X axis (in rad)
        self.sensor_angles = [
            0.5 * math.pi,   # Left perpendicular
            0.25 * math.pi,  # Left angled
            0.0,             # Left forward
            0.0,             # Right forward
            1.75 * math.pi,  # Right angled
            1.5 * math.pi,   # Right perpendicular
            math.pi,         # Back right
            math.pi          # Back left
        ]

        # Physical (x,y) offset of the sensor from the center of the bot in mm
        # Where x is forward, y is left lateral
        self.sensor_offsets = [
            ( 15.0,  25.0),  # Left perpendicular
            ( 20.0,  20.0),  # Left angled
            ( 27.0,   8.0),  # Left forward
            ( 27.0,  -8.0),  # Right forward
            ( 20.0, -20.0),  # Right angled
            ( 15.0, -25.0),  # Right perpendicular
            (-26.0, -10.0),  # Back right
            (-26.0,  10.0)   # Back left
        ]


    def sub_mapgen(self):
        '''
        Subscribe (so run async in a different process) to state updates published
        to Redis and affect the map
        '''
        sub = self.ds.r.pubsub()
        sub.subscribe([self.ds.listname])

        keys = set(['x','y','theta','t', 'r0','r1',
                    'r2','r3','r4','r5','r6','r7'])

        for msg in sub.listen():

            if msg['type'] == "subscribe":
                print("Subscribed successfully. {}".format(msg))
                continue

            try:
                if msg['channel'] == self.ds.listname:
                    
                    data = self.ds.r.hgetall(msg['data'])
                    
                    if not keys.issubset(set(data.keys())):
                        raise KeyError("Incomplete hashmap published '{}' --> {}"
                                       "".format(msg['data'], data))

                    # TODO: Do the calcs and push to map
                    
            except KeyError as e:
                print("!!!!! EXCEPTION - Continuing. {}".format(str(e)))
    


    def _activation_to_gridup(self, data):
        '''
        Arguments:
        data  --  hashmap of pose and distance sensor activations
        '''
        
        keys = ['r0','r1','r2','r3','r4','r5','r6','r7']
        pre_points = zip(keys, self.sensor_angles, self.sensor_offsets)

        points = []
        intensities = []

        # Basic trig, converts ranges to points relative to robot
        for point in pre_points:
            reading = float(data[point[0]])
            distance = utils.ir_to_dist(reading)

            #print(str(point[0]) + " at " + str(distance))

            # Don't render 'infinite' distance
            if distance > 60.0:
                continue
            
            pt = Point()

            # point[2] is the sensor's coords relative to the robot
            # point[1] is the angle the sensor takes relative to the robot's x axis

            pt.x = (distance * math.cos(point[1])) + point[2][0]
            pt.y = (distance * math.sin(point[1])) + point[2][1]

            intensities.append(distance)
            points.append(pt)


if __name__ == "__main__":


    args = sys.argv[1:]

    try:
        optlist, args = getopt.getopt(args, 's:', ['server='])
    except getopt.GetoptError:
        print("Invalid Option, correct usage:")
        print("-s or --server   : Hostname of redis server to use. Default localhost")
        sys.exit(2)

    server = "localhost"

    # Pre-instantiation options
    for opt, arg in optlist:
        if opt in ('-s', '--server'):
            print("Using Redis server at '" + str(arg) + "'")
            server = str(arg)

    m = Mapping()


