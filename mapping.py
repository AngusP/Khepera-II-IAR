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
    'r0': '1200.0',
    'r1': '1200.0',
    'r2': '20.0',
    'r3': '20.0',
    'r4': '1200.0',
    'r5': '20.0',
    'r6': '1200.0',
    'r7': '20.0',
    't': '1478446510.160574',
    'theta': '',
    'x': '161.64094776032476',
    'y': '72.77630028283815'
}


class Point():
    
    def __init__(self, x=None, y=None, val=None):
        '''
        Point with a value.
        Built-in methods will all operate over val
        '''
        self.x = x 
        self.y = y
        self.val = val

    def _euc(self, p):
        '''
        Euclidean distance
        '''
        dx = self.x - p.x
        dy = self.y - p.y
        return math.sqrt(dx**2 + dy**2)

    def __float__(self):
        return self.val

    def __int__(self):
        return int(float(self))

    def __eq__(self, p):
        return self.val == p.val

    def __ge__(self, p):
        return int(self) >= int(p)

    def __le__(self, p):
        return int(self) <= int(p)

    def __gt__(self, p):
        return int(self) > int(p)

    def __lt__(self, p):
        return int(self) < int(p)

    def __str__(self):
        return "({},{}) {}".format(self.x, self.y, self.val)

    __repr__ = __str__


class Mapping(object):

    def __init__(self, host='localhost', ds=None):
        
        if ds is None:
            self.ds = DataStore(host=server)
        else:
            self.ds = ds

        # Angles of the sensor from the X axis (in rad)
        self.sensor_angles = [
            0.5 * math.pi,   # Left perpendicular
            0.25 * math.pi,  # Left angled
            0.0,             # Left forward
            0.0,             # Right forward
            -0.25 * math.pi, #1.75 * math.pi,  # Right angled
            -0.5  * math.pi, #1.5 * math.pi,   # Right perpendicular
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

        TODO: FIX to match new message serialised format
        '''
        sub = self.ds.r.pubsub()
        sub.subscribe([self.ds.listname])

        keys = set(['x','y','theta','t', 'r0','r1',
                    'r2','r3','r4','r5','r6','r7'])

        try:
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
                        
                        self.update(data)

                except KeyError as e:
                    print("!!!!! EXCEPTION - Continuing. {} raised {}".format(msg, str(e)))
            
        except KeyboardInterrupt:
            print("Done, stopping...")



    def update(self, data):
        '''
        Apply updates to map from published hints.
        NOTE THAT key checking won'y be done here, that's your job dear reader.

        Arguments:
        data  --  full statestream pose and ranges
        '''
        points = self._activation_to_points(data)

        # Robot's _absolute_ position:
        x, y = map(float, [data['x'], data['y']])
        pointsl = set()

        # We can immediately declare the spot we're in is unoccupied:
        pointsl.add((x,y,0))

        priors = []
        
        for point in points:
            
            prior = self.ds.og.get(point.x, point.y)
            if prior is None:
                prior = 0

            # Ray-trace and update points between us 
            # and the object we're seeing
            spaces = self.raytrace((x,y), (point.x, point.y))
            priors = self.ds.og.mget(spaces)

            for space , prior in zip(spaces, priors):
                sx, sy = (space[0], space[1])

                prior = self.ds.og.get(sx, sy)
                
                if prior is None:
                    # Never seen before then assert it's unoccupied
                    pointsl.add((sx, sy, 0))
                else:
                    # TODO: More sensible update
                    # Reduce the prior as we now think it's unoccupied
                    pointsl.add((sx, sy, prior/2))
            
            if point.val > 70.0:
                occ = 0
            else:
                occ = 130 - (point.val * 1.5)
            
            # Basic assurance that we're within [0..100]
            occ = max(0, min(100, occ))
            pointsl.add((point.x, point.y, occ))
            
        pointsl = list(pointsl)
        self.ds.og.multiupdate(pointsl)
        
        return pointsl



    def raytrace(self, coord1, coord2, clamp=None):
        '''
        2D ray-tracing

        Return a list of all points between given coordinate tuples,
        to the granular scale

        Implementation of Bresenham's Line Algorithm

        Arguments:
        coord1  --  (x,y) tuple of starting point
        coord2  --  (x,y) tuple of stopping point
        clamp   --  Length of longest ray to return (list len) (Optional)
        '''
        
        x1, y1 = map(self.ds.og._snap, coord1)
        x2, y2 = map(self.ds.og._snap, coord2)
        
        x1, x2, y1, y2 = map(lambda x: int(x/self.ds.og.granularity), (x1,x2,y1,y2))

        # print("Taking ({},{}) to ({},{})".format(x1,y1,x2,y2))

        dx = x2 - x1
        dy = y2 - y1
        steep = abs(dy) > abs(dx)

        if steep:
            x1, y1 = y1, x1
            x2, y2 = y2, x2

        swapped = False
        if x1 > x2:
            x1, x2 = x2, x1
            y1, y2 = y2, y1
            swapped = True

        # Recalculate gradients
        dx = x2 - x1
        dy = y2 - y1

        # print("dy = {} dx = {}".format(dy, dx))
        
        # Calculate error
        error = int(dx / 2.0)
        ystep = 1 if y1 < y2 else -1
        
        
        # Iterate over bounding box generating points between start and end
        y = y1
        points = []
        
        # Only imbetween points, not end or start
        for x in xrange(x1+1, x2-1):
            coord = (y, x) if steep else (x, y)
            points.append((float(coord[0]) * self.ds.og.granularity,
                           float(coord[1]) * self.ds.og.granularity))
            error -= abs(dy)
            if error < 0:
                y += ystep
                error += dx

        if swapped:
            points.reverse()

        if clamp is not None:
            return points[:clamp]

        return points



    def predict_sensor(self, pose):
        '''
        Return anticipated sensor readings given a pose and the occupancy grid.

        Arguments:
        pose  --  (x,y,theta) pose tuple
        '''
        x, y = self.ds.og._snap(pose[0]), self.ds.og._snap(pose[1])
        theta = pose[2]

        angles = map(lambda t: t + theta, self.sensor_angles)
        sensors = map(lambda pt: utils.relative_to_fixed_frame_tf(x, y, theta, pt[0], pt[1]), 
                      self.sensor_offsets)

        pre_points = zip(sensors, angles)
        points = set()
        
        for point in pre_points:
            p, t = point
            points.update(set(self.angle_raytrace(p, t, 6)))

        points = list(points)
        occs = self.ds.og.mget(points)

        ret = []
        for occ, point in zip(occs, points):
            if occ > 0:
                ret.append((point[0], point[1], occ))

        return ret



    def angle_raytrace(self, start, theta, num):
        '''
        Similar to raytrace, though takes a start point and angle, returns
        a list points on the resulting ray.

        Arguments:
        start  --  (x,y) coordinate to start projecting from
        theta  --  Angle to project
        num    --  Number of points wanted
        '''

        xstep = math.cos(theta) * self.ds.og.granularity
        ystep = math.sin(theta) * self.ds.og.granularity

        # print("xs {} ys {}".format(xstep, ystep))

        endx = (xstep * (num+4) * math.sqrt(2))
        endy = (ystep * (num+4) * math.sqrt(2))

        return self.raytrace(start, (endx, endy), num)





    def _activation_to_points(self, data):
        '''
        Arguments:
        data  --  hashmap of pose and distance sensor activations

        Note no key existance checking will be done here

        Returns:
        Array of Points() instances, with coord and distance from bot
        '''
        
        keys = ['r0','r1','r2','r3','r4','r5'] #,'r6','r7']
        pre_points = zip(keys, self.sensor_angles, self.sensor_offsets)

        points = []

        # Basic trig, converts ranges to points relative to robot
        for point in pre_points:
            reading = float(data[point[0]])
            distance = utils.ir_to_dist(reading)
            # print("Reading {} yields dist {}".format(reading, distance))

            pt = Point()

            # point[2] is the sensor's coords relative to the robot
            # point[1] is the angle the sensor takes relative to the robot's x axis
            # We also have to transfer from the bot's reference frame to the fixed frame,
            # using the handy function provided by utils

            pt.x = (distance * math.cos(point[1])) + point[2][0]
            pt.y = (distance * math.sin(point[1])) + point[2][1]

            pt.x, pt.y = utils.relative_to_fixed_frame_tf(
                float(data['x']),
                float(data['y']), 
                float(data['theta']),
                pt.x, 
                pt.y)

            pt.val = distance

            points.append(pt)

        return points




if __name__ == "__main__":


    args = sys.argv[1:]

    try:
        optlist, args = getopt.getopt(args, 's:md:', ['server=', 'map', 'destroy_map'])
    except getopt.GetoptError:
        print("Invalid Option, correct usage:")
        print("-s or --server   : Hostname of redis server to use. Default localhost")
        print("-m or --map      : Run map generation program")
        print("-d or --destroy  : Destroy the map (not whole DB). Interactive.")
        sys.exit(2)

    server = "localhost"

    # Pre-instantiation options
    for opt, arg in optlist:
        if opt in ('-s', '--server'):
            print("Using Redis server at '" + str(arg) + "'")
            server = str(arg)

    m = Mapping(host=server)

    # Post-init
    for opt, arg in optlist:
        if opt in ('-m', '--map'):
            m.sub_mapgen()
        elif opt in('-d', '--destroy_map'):
            m.ds.og._destroy()
            sys.exit(0)


