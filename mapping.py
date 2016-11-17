#!/usr/bin/env/python

#
#     - - - -  I A R  - - - -
#
#  s1311631         Angus Pearson
#  s1346981    Jevgenij Zubovskij
#

from __future__ import print_function
from data import DataStore, DSException
import utils
import sys
import getopt
import math
import numpy as np
import random
import json

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


class Point(object):
    
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



#       ^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^



class Mapping(object):

    def __init__(self, host='/tmp/redis.sock', ds=None):
        
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
        NOTE THAT key existance checking won't be done here, 
        that's your job dear reader.

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
                    # Reduce the prior as we now think it's unoccupied
                    pointsl.add((sx, sy, prior * 0.9))

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

        Returns:

        List of (distance, occupancy) or if the ray did not hit anything, (None, None)
        for each sensor.
        '''

        x, y = self.ds.og._snap(pose[0]), self.ds.og._snap(pose[1])
        theta = pose[2]

        angles = map(lambda t: t + theta, self.sensor_angles)
        sensors = map(lambda pt: utils.relative_to_fixed_frame_tf(x, y, theta, pt[0], pt[1]), 
                      self.sensor_offsets)

        pre_points = zip(sensors, angles)
        ret = []

        for point in pre_points:
            p, t = point
            pixels = list(set(self.angle_raytrace(p, t, 6)))
            occs = self.ds.og.mget(pixels)
            min_dist = None
            this_occ = None
            for pixel, occ in zip(pixels, occs):
                if occ > 0:
                    d = self._euclidean((x,y), (pixel[0],pixel[1]))
                    if d < min_dist or min_dist is None:
                        min_dist = d
                        this_occ = occ

            ret.append((min_dist, this_occ))

        return ret


    def predict_msensors(self, poses):
        '''
        Same as singular predict_sensor(pose) but takes a list of poses
        and transacts with Redis for maximim speeds

        Arguments:
        poses  --  [(x,y,theta), ...]
        '''
        raise NotImplementedError()
        


    def transact_predict_sensors(self, poses):
        '''
        Same as singular predict_sensor(pose) but takes a list of
        poses. Executes as a single Redis transction for maximum 
        network speedz.

        Does some serious caching magic, and will look up the minimum
        set of keys required to make predictions

        Arguments:
        poses  --  [(x1,y1,theta1), ...]
        '''

        pipe = self.ds.r.pipeline()
        keys = []

        def cache_query(key):
            if key in keys:
                pass
            else:
                pipe.hget(key, 'occ')
                keys.append(key)

        for pose in poses:
            x, y = self.ds.og._snap(pose[0]), self.ds.og._snap(pose[1])
            theta = pose[2]
            
            angles = map(lambda t: t + theta, self.sensor_angles)
            sensors = map(lambda pt: utils.relative_to_fixed_frame_tf(x, y, theta, pt[0], pt[1]), 
                          self.sensor_offsets)
            
            pre_points = zip(sensors, angles)
            ret = []

            for point in pre_points:
                p, t = point
                pixels = self.angle_raytrace(p, t, 6)
                
                for pixel in pixels:
                    cache_query(self.ds.og._genkey(pixel[0], pixel[1]))

        # Pull from Redis
        cache = pipe.execute()

        assert len(cache) == len(keys), "Transaticon key and returned value lists "\
            "are different lengths! {} != {}".format(len(cache), len(keys))

        # print("cachemap transaction looked up {} keys".format(len(cache)))

        def safe_int_cast(val):
            try:
                return int(val)
            except (ValueError, TypeError, AttributeError):
                return None

        # Get it...?
        cachemap = dict()

        def into_hmap(cachepair):
            k, v = cachepair
            cachemap[k] = safe_int_cast(v)
        
        map(into_hmap, zip(keys, cache))
        
        mret = []

        for pose in poses:
            x, y = self.ds.og._snap(pose[0]), self.ds.og._snap(pose[1])
            theta = pose[2]
            
            angles = map(lambda t: t + theta, self.sensor_angles)
            sensors = map(lambda pt: utils.relative_to_fixed_frame_tf(x, y, theta, pt[0], pt[1]), 
                          self.sensor_offsets)
            
            pre_points = zip(sensors, angles)
            ret = []
            
            for point in pre_points:
                p, t = point
                pixels = self.angle_raytrace(p, t, 6)
                occs = []
                for pixel in pixels:
                    pixelkey = self.ds.og._genkey(pixel[0], pixel[1])
                    occs.append(cachemap[pixelkey])
                min_dist = None
                this_occ = None
                for pixel, occ in zip(pixels, occs):
                    if occ > 0:
                        d = self._euclidean((x,y), (pixel[0],pixel[1]))
                        if d < min_dist or min_dist is None:
                            min_dist = d
                            this_occ = occ
                            
                ret.append((min_dist, this_occ))
            mret.append(ret)

        return mret
        



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



    def _euclidean(self, a, b):
        '''
        Calculate the straight-line distance between two (x,y) tuple coordinates
        '''
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])
        return math.sqrt(dx**2 + dy**2)




#       ^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^




class Particle(object):
    
    def __init__(self, pose, weight):
        '''
        Point with a value.
        Built-in methods will all operate over val
        '''
        self.x, self.y, self.theta = pose
        self.weight = weight
        self.uid = self.__hash__()

    def __repr__(self):
        return "({},{},{}):{}".format(self.x, self.y, self.theta, self.weight)

    __str__ = __repr__

    def __iter__(self):
        yield self.x
        yield self.y
        yield self.theta




#       ^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^





class Particles(object):
    '''
    Adaptive Monte-Carlo Localisation (Particle Filter)
    '''

    def __init__(self, mapper, numparticles=100, initial_hypothesis=(0,0,0), noise=0):
        '''
        Initialise a particle filter (Monte-Carlo) localiser
        
        Arguments:
        numparticles        --  Number of particles to use - more is slower but likely more accurate
        initial_hypothesis  --  Starting hypothesis three-tuple (x,y,theta)
        noise               --  Standard deviation of normally distributed noise to add to initial hyp
        '''
        self.m = mapper
        self.particles = []
        
        self.control = None
        self.dt = None

        #Redis channel
        self.partchan = "particlestream"

        # Hyperparameter (aka magic number)
        self.DRIFT_SMOOTHING = 300.0
        self.DRIFT_ROTATIONAL_SMOOTHING = 2.0
        '''
        DRIFT_SMOOTHING is the reciportical multiplier;
        higher numbers reduce the magnitude of the change.
        Really important to how quickly hypotheses
        diverge.
        '''
        
        # Stops Python being weird and copying reference
        for i in xrange(numparticles):
            hyp = initial_hypothesis
            if noise != 0:
                hyp = map(lambda x: x + np.random.normal(0,int(noise)), hyp)
            self.particles.append(Particle(hyp, 1))
        
        self.__gauss_base = 1.0/(math.sqrt(2*math.pi)*0.5)


    def __str__(self):
        return "{} Particles : {}".format(len(self.particles), self.particles)


    def __iter__(self):
        '''
        Iterate over particles
        '''
        for particle in self.particles:
            yield particle


    def __getitem__(self, key):
        '''
        Allows indexing the class
        '''
        return self.particles.__getitem__(key)


    def __len__(self):
        '''
        Get number of particles (len)
        '''
        return self.particles.__len__()


    def push_params(self, dt, ds, dtheta, readings):
        '''
        Give the Particle Filter the change in time
        since it was last run and the control change

        Arguments:
        dt        --  Delta time, seconds
        ds        --  Delta displaement, 's'
        dtheta    --  Delta rotation, 'theta'
        readings  --  Array of 8 sensor readings (raw)
        '''
        self.dt = dt
        self.control = (ds/dt, dtheta/dt) # Velocity, angular velocity
        self.sensor_readings = readings


    def whereami(self):
        '''
        USE THIS ONE
        
        Returns the pose of the robot derived from
        the particles
        '''
        avg_x = 0
        avg_y = 0
        avg_theta = 0
        sum_wgts = 1e-10

        for p in self:
            avg_x += p.x * p.weight
            avg_y += p.y * p.weight
            avg_theta += p.theta * p.weight
            sum_wgts += p.weight
        
        return (avg_x/sum_wgts, avg_y/sum_wgts, avg_theta/sum_wgts), sum_wgts/len(self)

        

    def update(self):
        '''
        Main particle localisation godmethod
        '''
        if self.dt is None or self.control is None:
            raise DSException("Tried to update particles before calling push_params()")
        
        v, w = self.control
        v_dt = v * self.dt
        w_dt = w * self.dt

        sigma = (math.sqrt(v**2 + w**2)/self.DRIFT_SMOOTHING) * self.dt
        sigma += 0.0001

        def motion_update(particle):
            '''
            Given a control input, move the pixels
            
            Arguments:
            particle  -- Particle() instance
            '''
            x, y, theta = tuple(particle)

            new_pose = (x + np.random.normal(v_dt * math.cos(theta), scale=sigma),
                        y + np.random.normal(v_dt * math.sin(theta), scale=sigma),
                        theta - np.random.normal(w_dt, scale=sigma/self.DRIFT_ROTATIONAL_SMOOTHING))
            return new_pose


        def sensor_update(particles):
            '''
            For all particles, in-place updates weight
            '''
            predictions = self.m.transact_predict_sensors(map(tuple, particles))
            for particle, predicted in zip(particles, predictions):
                particle.weight = self.sensor_likelihood(self.sensor_readings, 
                                                         predicted)
        
        for p in self.particles:
            p.x, p.y, p.theta = motion_update(p)
    
        sensor_update(self.particles)

        newparticles = []

        for i in xrange(len(self)):
            r, i = self.weighted_random_sample()
            newparticles.append(Particle((r.x, r.y, r.theta), r.weight))

        self.particles = newparticles
        
        # Push new particles to Redis
        pipe = self.m.ds.r.pipeline()
        pipe.delete(self.partchan)

        for i, particle in enumerate(self):
            pipe.lpush(self.partchan, json.dumps(list(particle)))

        # Announce new particles are available
        pipe.publish(self.partchan, "{} {}".format(self.partchan, len(self.particles)))
        pipe.execute()

        return self.whereami()



    def __call__(self):
        '''
        The Particles class is callable, which will incur an update
        '''
        return self.update()



    def sensor_likelihood(self, reading, predicted):
        '''
        Returns the likelihood a given set of observations is compared
        to a predicted set from the map.

        Arguments:
        reading    --  List of N dimension actual readings
        predicted  --  N dimensional list of tuple redicted readings (rdg, occ)
        '''
        if len(reading) != len(predicted):
            raise TypeError("Dimensionality mismatch, got arg1 {} arg2 {}"
                            "".format(len(reading), len(predicted)))
        
        l = 0
        for r, p_withocc in zip(reading, predicted):
            p, occ = p_withocc

            if p is None or occ is None:
                # Here we have no prediction
                # so we assign an average probability
                l += 0.5
                continue

            if occ != -1:
                mul = occ / 100.0
            else:
                mul = 1.0

            l += self._gaussian_p(r,p) * mul

        # Average all likelihoods
        return l / len(reading)



    def weighted_random_sample(self):
        '''
        Using Weighted Reservoir Sampling Algorithm
        Return randomly sampled particle.

        Due to weights, some bias against first particle exists
        '''
        keep = self[0]
        keepth = 0
        
        for i, particle in enumerate(self):
            weight = abs(i + 2 + 1.0/(particle.weight + 1e-8))
            
            if random.randint(0, int(weight)) == 0:
                keep = particle
                keepth = i

        return keep, keepth


    def _gaussian_p(self, s_reading, p_reading):
        '''
        Gaussian Probability

                                      1                (-{[x-mean]^2}/{2var^2})
        p(x | mean, var) =  ---------------------- * e^
                             sqrt(2 * var^2 * Pi)
        '''
        
        mu = p_reading - s_reading
        exp = -(mu**2)/0.5 
        return self.__gauss_base * math.exp(exp)



#       ^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^


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

    server = "/tmp/redis.sock"

    # Pre-instantiation options
    for opt, arg in optlist:
        if opt in ('-s', '--server'):
            print("Using Redis server at '" + str(arg) + "'")
            server = str(arg)

    m = Mapping(host=server)
    pf = Particles(m)

    # Post-init
    for opt, arg in optlist:
        if opt in ('-m', '--map'):
            m.sub_mapgen()
        elif opt in('-d', '--destroy_map'):
            m.ds.og._destroy()
            sys.exit(0)


