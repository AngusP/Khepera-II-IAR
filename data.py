#!/usr/bin/env/python

#
#     - - - -  I A R  - - - -
#
#  s1311631         Angus Pearson
#  s1346981    Jevgenij Zubovskij
#

from __future__ import print_function
import redis
import whiptail
from state import *
import sys
import getopt
import matplotlib.pyplot as plt
import math
import time
import yaml
import pprint
import re

import constants
import utils




# Check for ROS (http://ros.org/) support
try:
    import rospy
    ros = True
    from std_msgs.msg import String
    from geometry_msgs.msg import PoseWithCovariance, PoseStamped, Point32
    from nav_msgs.msg import Odometry, Path, OccupancyGrid
    from sensor_msgs.msg import PointCloud, ChannelFloat32
    import tf
except ImportError as e:
    print(e)
    print("[NOTICE] Continuing without ROS integration")
    ros = False
    del e


# ROS Import check wrapper
def requireros(func):
    def check_and_call(*args, **kwargs):
        if not ros:
            raise ImportError("[FATAL] ROS (rospy) Not supported/found")
        else:
            return func(*args, **kwargs)
    return check_and_call




# Check for Pillow (Python Imaging Oibrary)
try:
    import PIL
    import numpy as np
    pil = True
except ImportError as e:
    print(e)
    print("[NOTICE] Continuing withut PIL support")
    pil = False
    del e


# PIL Import check wrapper
def requireimage(func):
    def check_and_call(*args, **kwargs):
        if not pil:
            raise ImportError("[FATAL] PIL (Image Lib) Not supported/found")
        else:
            return func(*args, **kwargs)
    return check_and_call





#       ^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^



class DataStore:

    '''
    Primary Data Munging class, handles chatter with Redis and ROS.
    `data.py` can be used interactively.
    
                   _._    
              _.-``__ ''-._
         _.-``    `.  `_.  ''-._   
     .-`` .-```.  ```\/    _.,_ ''-._ 
    (    '      ,       .-`  | `,    )
    |`-._`-...-` __...-.``-._|'` _.-'|
    |    `-._   `._    /     _.-'    |
     `-._    `-._  `-./  _.-'    _.-' 
    |`-._`-._    `-.__.-'    _.-'_.-'|
    |    `-._`-._        _.-'_.-'    |
     `-._    `-._`-.__.-'_.-'    _.-' 
    |`-._`-._    `-.__.-'    _.-'_.-'|
    |    `-._`-._        _.-'_.-'    |
     `-._    `-._`-.__.-'_.-'    _.-'
         `-._    `-.__.-'    _.-'
             `-._        _.-'
                 `-.__.-'


    '''

    def __init__(self, host='localhost', db=0, port=6379):
        '''
        Instantiate a DataStore class

        Arguments:
        host  --  Redis server to use, defaults to 'localhost'
        db    --  Redis database to use, default 0
        port  --  Port the Redis server is listening on

        Instantiates it's own StrictRedis, PrettyPrint and Whiptail classes
        '''
        self.r = redis.StrictRedis(host=host, port=port, db=db)
        self.wt = whiptail.Whiptail()
        self.pp = pprint.PrettyPrinter(indent=1)

        self.og = GridManager(self.r, debug=True)

        # Redis List and Channel name
        self.listname = "statestream"
        self.goallist = "goalstream"
        self.mapchan  = self.og.channel
        # ROS Topics
        self.posetopic = self.listname + "pose"
        self.odomtopic = self.listname + "odom"
        self.disttopic = self.listname + "dist"
        self.goaltopic = self.listname + "goal"
        self.maptopic  = self.listname + "map"

        # Test Redis connection
        if not self.r.ping():
            raise redis.ConnectionError("No PONG from PING!")

    def __str__(self):
        '''
        return a string representation of self
        '''
        return self.pp.pformat(self.__dict__)

    __repr__ = __str__

    def __del__(self):
        self.save()


    def keys(self):
        '''
        Returns a list of all keys in the Redis datastore
        '''
        return self.r.keys()

    
    def push(self, pose, ranges=None):
        '''
        Push a new 'Pose' onto Redis, and optionally distance data.
        
        Arguments:
        pose    --  Either GenericState instance or a dict, must have 
                    x, y, t and theta members/keys
        ranges  --  List of raw sensor readings (float), length 8

        Pushed are added to the history list and also published to the 
        predefined Redis channels
        '''
        # Type duck between GenericState and a dict
        if isinstance(pose, GenericState):
            hmap = {
                't'    : pose.time,
                'x'    : pose.x,
                'y'    : pose.y,
                'theta': pose.theta
            }
        else:
            hmap = {
                't'    : pose['t'],
                'x'    : pose['x'],
                'y'    : pose['y'],
                'theta': pose['theta']
            }

        # Name we'll give this pose in Redis
        mapname = "pose-" + str(hmap['t'])
        
        # Build hashmap from given state class
        if ranges is not None and len(ranges) == 8:
            hmap.update({
                'r0' : float(ranges[0]),
                'r1' : float(ranges[1]),
                'r2' : float(ranges[2]),
                'r3' : float(ranges[3]),
                'r4' : float(ranges[4]),
                'r5' : float(ranges[5]),
                'r6' : float(ranges[6]),
                'r7' : float(ranges[7])
                })
        
        # For each part of the _python_ dict we
        # create a Redis hashmap using the time as index
        self.r.hmset(mapname, hmap)

        # We push a reference to this new hashmap onto 
        # the statestream list
        self.r.rpush(self.listname, mapname)

        # Also publish onto a channel
        self.r.publish(self.listname, mapname)


    def push_goal(self, path):
        '''
        Similar to push(), pushes a new goal path to Redis.

        Arguments:
        path --  A list of dicts e.g. [{'x': 0.0, 'y':100.0},{'x': 1502.5, 'y': 1337.0}]
        '''
        
        # Clear out old points
        for key in self.r.lrange(self.goallist, 0, -1):
            self.r.delete(key)
        # Delete old list
        self.r.delete(self.goallist)

        pointnum = 0
        for point in path:
            pointname = "goal" + str(pointnum)
            pointnum += 1
            self.r.hmset(pointname, point)
            self.r.rpush(self.goallist, pointname)

        self.r.publish(self.goallist, self.goallist)



    def push_map(self, og):
        '''
        Push a new map (occupancy grid)
        '''
        raise NotImplementedError()


    def sub(self):
        '''
        Mostly a test method, subscribe to Redis channels and print
        any messages that come over it.
        '''
        sub = self.r.pubsub()
        sub.subscribe([self.listname, self.goallist])

        # Loop until stopped plotting the path
        try:
            for item in sub.listen():
                if self.r.exists(item['data']):
                    data = self.r.hgetall(item['data'])
                    item['data'] = data
                self.pp.pprint(item)
        except KeyboardInterrupt as e:
            print("Stopping...")
            pass

        
    def get(self, start=0, stop=-1):
        '''
        Returns a list in-order over the range given in args. Default is
        to return the entire list.

        Arguments:
        start  --  First list index
        stop   --  Last list index, -1 means all
        '''
        keys = self.r.lrange(self.listname, int(start), int(stop))
        ret = list()

        # Pull all the hashmaps out of the store
        for key in keys:
            ret.append(self.r.hgetall(key))

        return ret

    
    def get_dict(self, start=0, stop=-1):
        '''
        Extends the behaviour of get(), provides a time-keyed
        dictionary of data as opposed to a list

        Arguments:
        start  --  First list index
        stop   --  Last list index, -1 means all
        '''
        # Return a dictionary instead of a list
        lst = self.get(start, stop)
        ret = dict()
        for item in lst:
            # time as key, dict as value
            ret[int(item['t'])] = item
            
        return ret

    
    def delete_before(self, time):
        '''
        Remove data with a key from earlier than specified from Redis
        
        Arguments:
        time  --  cutoff time, any value less than this will be killed
        '''
        if time < 0:
            raise ValueError("Time must be positive you crazy person!")

        keys = self.r.lrange(self.listname, 0, -1) # All keys in our stream
        
        for key in keys:
            if int(key) <= int(time):
                # Remove from list
                self.r.lrem(self.listname, count=0, value=key)
                # delete the key (hashmap)
                self.r.delete(key)


    def plot(self, start=0, stop=-1):
        self.static_plot(start, stop)


    def live_plot(self):
        print("Deprecated, rospipe + rviz will work better")
        try:
            pubsub = self.r.pubsub()
            pubsub.subscribe([self.listname])
            plt.axis([-500, 500, -500, 500])
            plt.ion()
            while True:
                # Loop until stopped plotting the path
                for item in pubsub.listen():
                    #print(item)
                    if self.r.hexists(item['data'], 'x'):
                        data = self.r.hgetall(item['data'])
                        plt.scatter(float(data['x']), float(data['y']))
                        #print(str(data['x']) + " " + str(data['y']))
                        plt.show()
                        plt.pause(0.0001)
                
        except KeyboardInterrupt as e:
            print(e)

            
    def static_plot(self, start=0, stop=-1):
        '''
        Produces a matplotlib plot of all odometry data in redis.

        Arguments:
        start  --  First list index
        stop   --  Last list index, -1 means all
        '''
        data = self.get(start, stop)
        
        xs = []
        ys = []
        
        for point in data:
            xs.append(point['x'])
            ys.append(point['y'])
            
        plt.axis([-600, 600, -600, 600])
        plt.ion()
        plt.plot(xs, ys)
        plt.show()


    @requireros
    def rospipe(self):
        '''
        Pipes messages published on Redis channels to ROS Topics, 
        transforming parts of the data allowing other stuff that
        speaks ROS to interact with the data.
        '''

        print('''
    __    __    __      _______     _____     _____    
   /  \  /  \  /  \    |  ____ \   / ___ \   / ____|   
   \__/  \__/  \__/    | |    \ | | /   \ | | |        
    __    __    __     | |    | | | |   | | | \____    
   /  \  /  \  /  \    | \___/ /  | |   | | \_____ \   
   \__/  \__/  \__/    |  __   \  | |   | |       \ |  
    __    __    __     | |   \  | | |   | |       | |  
   /  \  /  \  /  \    | |    | | | \___/ |  ____/  |  
   \__/  \__/  \__/    |_|    |_|  \_____/  |______/   
''')

        print("Piping Redis ---> ROS")
        # Pipe data out of Redis into ROS

        rg = ROSGenerator()
        
        pose_pub = rospy.Publisher(self.posetopic, PoseStamped, queue_size=100)
        odom_pub = rospy.Publisher(self.odomtopic, Odometry, queue_size=100)
        dist_pub = rospy.Publisher(self.disttopic, PointCloud, queue_size=100)
        goal_pub = rospy.Publisher(self.goaltopic, Path, queue_size=10)
        map_pub  = rospy.Publisher(self.maptopic,  OccupancyGrid, queue_size=100)
        tbr = tf.TransformBroadcaster()

        # Pre-subscription, we should load the OccupancyGrid Map.
        # This'll be the only one of our ROS classes that persists
        print("generating map...")
        og_map = rg.gen_map(self.og)
        print("done.")

        sub = self.r.pubsub()
        sub.subscribe([self.listname, self.goallist, self.mapchan])

        map_pub.publish(og_map)

        # Loop until stopped plotting the path
        for item in sub.listen():

            if rospy.is_shutdown():
                print("\nStopping rospipe...")
                break

            if item['type'] == "subscribe":
                rospy.loginfo("Subscribed successfully to " + item['channel'])
                continue
            
            try:
                if item['channel'] == self.listname:
                    # Stream of poses and distance data

                    # Pull from redis:
                    data = self.r.hgetall(item['data'])
                    
                    required_keys = ['x','y','theta','t']
                    for key in required_keys:
                        if key not in data.keys():
                            raise KeyError("Missing key " + str(key) + " from hashmap " +
                                           str(data) + " on channel " + str(item['channel']))

                    # Generate a new Quaternion based on the robot's pose
                    quat = tf.transformations.quaternion_from_euler(0.0, 0.0, 
                                                                    float(data['theta']))

                    # Generate new pose
                    pose = rg.gen_pose(data, quat)
                    pose_pub.publish(pose)

                    # Publish Khepera transform
                    tbr.sendTransform((pose.pose.position.x, pose.pose.position.y, 0),
                                      quat, rospy.Time.now(), "khepera", "map")
                
                    # Generate odometry data
                    odom = rg.gen_odom(data, quat)
                    odom_pub.publish(odom)

                    # Generate pointcloud of distances
                    dist = rg.gen_dist(data)
                    dist_pub.publish(dist)

                    rospy.loginfo(" Redis " + str(item['channel']) + " --> ROS")


                elif item['channel'] == self.goallist:
                    # Less frequent planning channel
                
                    # Pull co-ords from Redis
                    data = self.r.lrange(self.goallist, 0, -1)
                    path = rg.gen_path()
                    quat = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)

                    for datapoint in data:
                        raw_pose = self.r.hgetall(datapoint)
                        
                        required_keys = ['x','y']
                        for key in required_keys:
                            if key not in raw_pose.keys():
                                raise KeyError("Missing key " + str(key) + " from point " +
                                               str(raw_pose) + " on channel " + 
                                               str(item['channel']))

                        
                        this_pose = rg.gen_pose(raw_pose, quat)
                        path.poses.append(this_pose)
                        
                    goal_pub.publish(path)
                    rospy.loginfo(" Redis " + str(item['channel']) + " --> ROS")


                elif item['channel'] == self.mapchan:
                    # Handle a map diff (update)
                    # TODO: Handle dimension changes in a less slow way
                    # TODO: Handle granularity changes                  
                    # A sharp represents a meta message
                    if not item['data'].startswith("#"):

                        x, y = self.og._dekey(item['data'])
                        
                        occ = self.og.get(x,y)
                        
                        if occ is None:
                            raise KeyError("Published key {} had no occupancy".format(item['data']))
                        
                        # in-place update this grid in og_map (the ROS OccupancyGrid instance)
                        # the map takes absolute coordinates to array index
                        x, y = self.og._genindex(x, y)
                        width, height = self.og._get_map_dimensions()
                        y *= width # row major, so scale y onto a flat array
                        og_map.data[int(x+y)] = occ # Toootaly worked first time

                        rospy.loginfo("Map update {} --> {}".format(item['data'], occ))


                    else:
                        if item['data'].startswith("#bounds"):
                            new_bounds = yaml.load(item['data'].strip("#bounds"))
                            
                            if type(new_bounds) is not dict:
                                raise TypeError("Couldn't deserialise {} message '{}' into dict - got {}"
                                                "".format(item['channel'], item['data'], type(new_bounds)))
                            
                            # TODO: In-place update, waaaay faster
                            # 'bounds check' with True, which will resize the 
                            self.og._bounds_check(new_bounds['minx'], new_bounds['miny'], True)
                            self.og._bounds_check(new_bounds['maxx'], new_bounds['maxy'], True)
                            
                            # SLOOOOOOOOW, maybe blocking subscribers
                            rospy.logerr("MAP RELOADING")
                            rospy.logwarn("Update occurred outwith bounding box, reloading map")
                            rospy.logwarn("This will take a while...")
                            og_map = rg.gen_map(self.og)
                            rospy.logwarn("DONE WITH RELOAD")

                    # ALways refresh
                    map_pub.publish(og_map)

                else:
                    # If we get an unexpected channel, complain loudly.... Thish should never happen
                    complaint = "Encountered unhandleable channel '" + str(item['channel']) + "'"
                    raise ValueError(complaint)


            except (KeyError, ValueError, IndexError, TypeError) as e:
                rospy.logwarn(str(e))
                #raise e


    def replay(self, speed=1.0, limit=-1):
        '''
        Replay data already stored, by re-publishing to the Redis channel
        
        Arguments:
        speed  --  Multiplier for replay speed, 1.0 is real time, > 1 is faster. Default 1.0
        limit  --  How far back to look (number of epochs) default -1 (all)
        '''

        print('''
      ____________________________
    /|............................|
   | |:      KHEPERA REWIND      :|
   | |:       "Redis & Co."      :|
   | |:     ,-.   _____   ,-.    :|
   | |:    ( `)) [_____] ( `))   :|
   |v|:     `-`   ' ' '   `-`    :|
   |||:     ,______________.     :|
   |||...../::::o::::::o::::\.....|
   |^|..../:::O::::::::::O:::\....|
   |/`---/--------------------`---|
   `.___/ /====/ /=//=/ /====/____/
        `--------------------'

   ///                         /#//#/
 ///-------------------------/#/-/#/
 \\\\\-------------------------\\#\\-\\#\\ 
   \\\\\                         \\#\\\\#\\ 
''')
        data = self.r.lrange(self.listname, 0, limit)
        
        try:
            for epoch in data:
                print("epoch " + str(epoch))
                self.r.publish(self.listname, epoch)
                time.sleep(constants.MEASUREMENT_PERIOD_S * speed)
                
        except KeyboardInterrupt as e:
            print(e)

        print("Replay done.")


    def _destroy(self):
        '''
        Clear out all (literally all) data held in Redis
        by flushing all keys from the DB. Interactive (uses whiptail prompt)
        '''
        if self.wt.confirm("Really destroy all data in Redis store?\n\nThis is not undoable!\n(run FLUSHDB)",
                           default='no'):
            self.r.flushdb()
            print("!!! Flushed Redis Data Store !!!")
        else:
            print("Did not flush Redis Store")


    def save(self):
        '''
        Copy DB to disk
        '''
        return self.r.save()








#       ^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^



class GridManager:

    '''
    Handles the Occupancy Grid's mechanics, as it's a more complex datastructure.
    Gets passed a reference to (presumably) a `DataStore`'s Redis instance.

    
    Origin is centered in the map.
    The Map is _sparse_, in that it is hypothetically infinitely large.
    '''
    
    def __init__(self, redis, granularity=10.0, debug=False):
        '''
        Arguments:
        granularity  --  Minimum distance representable in the map, as a decimal multiple of 
                         whatever units the distances are given in.

        Note: Very small granularities and very large granularities that may have '10e-12' 
        representations by default will break as they cannot be correctly keyed and de-keyed.

        Standards & Conventions:
        map:x:y = {
            'occ' : -1 or [0..100]
        }


        Axes:

                 (forward)
                     X
                     ^
                     |
                     |
                     |
          Y <------- Z
        (left)      (up)
        '''
        self.DEBUG = debug

        # Origin is bottom left corner, x left, y up
        self._testworld = """\
????????????????????????????????
?XXXXXXXXXXXXXXXXXXXXXXXXXXXXXX?
?X                  XX    XX  X?
?X                   X    XX  X?
?X     XXXXXXX       XX       X?
?X     XXXXXXX        X    XXXX?
?X     XX                  X  X?
?X     XX                     X?
?X                            X?
?X              XX            X?
?X             XXXX           X?
?X              XX            X?
?X    XX                      X?
?X   XXXX               XXXXXXX?
?X    XX     XX         X     X?
?X           XX         X     X?
?X           XX         X     X?
?X           XX               X?
?X   XXXX    XX               X?
???? XXXX          XXXX       ??
????        ?      X??X       ??
??????      ????   XXXX      ???
?????        ????            ???
??????         ??? ?????    ????
???????             ?????? ?????
????????          ??????????????
???????   ??     ???????????????
????????????     ???????????????
????????????????      ??????????
?????????????????    ???????????
????????????????????????????????
????????????????????????????????
????????????????????????????????
????????????????????????????????
"""
        
        self.granularity = granularity
        
        self.wt = whiptail.Whiptail()
        self.pp = pprint.PrettyPrinter(indent=1)

        self.r = redis
        # Redis list and channel names
        self.mapname = "map"
        self.mapmeta = self.mapname + "-meta"
        self.channel = self.mapname + "-update"
        
        # Default origin (location of the map)
        # wit respect to the fixed reference frame
        self.origin = {
            'x'       : 0.0,
            'y'       : 0.0,
            'z'       : 0.0,
            'quat_x'  : 0.0,
            'quat_y'  : 0.0,
            'quat_z'  : 0.0,
            'quat_w'  : 0.0
        }

        # Default size
        self.bounds = {
            'maxx'    :  1000.0,
            'maxy'    :  1000.0,
            'minx'    : -1000.0,
            'miny'    : -1000.0
        }

        # If prior config exists, pull it in
        for k, v in self.origin.iteritems():
            if self.r.hexists(self.mapmeta, k):
                self.origin[k] = float(self.r.hget(self.mapmeta, k))
                if self.DEBUG:
                    print("{} got {} = {} from Redis".format(self.__class__.__name__, k, self.origin[k]))

        for k, v in self.bounds.iteritems():
            if self.r.hexists(self.mapmeta, k):
                self.bounds[k] = float(self.r.hget(self.mapmeta, k))
                if self.DEBUG:
                    print("{} got {} = {} from Redis".format(self.__class__.__name__, k, self.bounds[k]))

            
        if self.r.hexists(self.mapmeta, "granularity"):
            self.granularity = float(self.r.hget(self.mapmeta, "granularity"))
            print("{} got granularity = {} from Redis".format(self.__class__.__name__, self.granularity))

        # Push origin and boundaries back to redis
        self.r.hmset(self.mapmeta, self.origin)
        self.r.hmset(self.mapmeta, self.bounds)
        self.r.hmset(self.mapmeta, {
            "granularity" : self.granularity
        })

        if self.DEBUG:
            self.pp.pprint(self.bounds)
            print("Using redis key '{}' for map, channel '{}' for updates"
                  "".format(self.mapname, self.channel))


        # Regex to turn a serialised key into coords
        # This re finds a combination of two floats or two ints
        # IMPORTANT: Must remain consistent with _genkey()
        self._dekey_re = re.compile("([+-]?\d+\.\d*|[+-]?\d+):([+-]?\d+\.\d*|[+-]?\d+)$")




    def __str__(self):
        '''
        return a string representation of self
        '''
        return self.pp.pformat(self.__dict__)

    __repr__ = __str__



    def get(self, x, y):
        '''
        Returns the int held at the given grid coord's occupancy value

        Handles type expetions, snaps to grid
        
        BE AWARE coords will silently be snapped to the grid, allowing
        querying at higher resolution than is represented in the map.
        '''
        return self._get_keyed(self._genkey(x, y))


    def _get_keyed(self, k):
        '''
        Similar to get(), though takes a string. Suggested you don't use
        this much as it is breakier. Almost always generate k with 
        GridManager._genkey(str)

        Arguments:
        k  --  str, name of redis key of grid
        '''
        occ = self.r.hget(k, 'occ')
        try:
            return int(occ)
        except (ValueError, TypeError, AttributeError):
            return None



    def get_map(self, rtype='L'):
        '''
        Return an array representing the whole map. Occupancies are ints, 
        default is unknown (-1). Range should be [-1..100].

        Arguments:
        rtype  --  Type to return, default 'L'
                -  'L' is a 2D python row-major list (biggest me use)
                -  'N' is a 2D row-major NumPy ndarray
                -  'C' is a flattened row-major NumPy ndarray (C style)
                -  'F' is a flattened row-major NumPy ndarray (C style)
        

        NOTE: This method returns a fully populated map wthin the bounding
              box, it is *not* going to be as fast.
              For speed use methods such as get() and Redis subscribers to 
              in-place update a cached map.
        '''
        xwidth, yheight = self._get_map_dimensions()
        data = np.ndarray((yheight, xwidth), dtype=int)
        data.fill(-1)

        for k in self._get_map_keys():
            x, y = self._dekey(k)
            xi, yi = self._genindex(x, y)
            data[yi][xi] = self._get_keyed(k)

        if rtype == 'N':
            return data
        elif rtype == 'C':
            return data.flatten('C') # C/C++ Style, row major
        elif rtype == 'F':
            return data.flatten('F') # FORTRAN Style, col major
        
        # Default ('L') give a 2D list
        return data.tolist()



    def _get_map_keys(self):
        '''
        Super simple, dump raw 100% organic map Redis data at your face
        '''
        return self.r.smembers(self.mapname)



    def _get_map_dimensions(self):
        '''
        Returns array size needed to fit map given bounds and granularity.
        '''
        xwidth  = 1 + int(math.ceil((-self.bounds['minx'] + self.bounds['maxx']) / self.granularity))
        yheight = 1 + int(math.ceil((-self.bounds['miny'] + self.bounds['maxy']) / self.granularity))
        return xwidth, yheight



    def _genindex(self, x, y):
        '''
        Given a coordinate on grid return the corresponding index into a 2D 
        array for that point, row major:
        
        [[ (0,0)     , (1,0)     , ... , (len(x),0)     ],
         ...
         [ (0,len(y)), (1,len(y)), ... , (len(x),len(y))]]
        
        This method is very similar to GridManager._get_map_dimensions(), which could
        be used to initialise an array thn indexed with this method.

        Arguments:
        x  --  x (forward) axis coord
        y  --  y (left) axis coord
        '''

        xoffset = math.floor(-self.bounds['minx'] / self.granularity)
        yoffset = math.floor(-self.bounds['miny'] / self.granularity)

        xi = int((self._snap(x) / self.granularity) + xoffset)
        yi = int((self._snap(y) / self.granularity) + yoffset)

        if xi < 0 or yi < 0:
            raise IndexError("Co-ordinates ({},{}) yielding index [{},{}] out of bounds; "
                             "Bounds are {}".format(x, y, xi, yi, self.bounds))

        return xi, yi


    def _snap(self, coord):
        '''
        Snap a given coordinate to the grid

        Snapping treats the occupancy grid squares as being centred on the pysical (e.g.)
        pose co-ordinate space: (Shown bracketed '(0,0)', OG shown braced '[0,0]')
        
                 Y
                 ^
                 |
         ________|________
        |        |        |
        |        |        |
        |        |        |
        |        |(0,0)   |
        |        \-----------> X
        |                 |
        |                 |
        | [0,0]           |
        \_________________/

        Arguments:
        coord  --  Arbuitary coordinate (float, int)

        Returns:
        coord  --  Snapped to grid. int or float, depending on whether the granularity is an
                   integer or float.
        '''
        # See how far it is from the next point
        distance = float(coord) % self.granularity
        
        # If closer to a higher one, push it up
        if distance >= self.granularity/2.0:
            coord += (self.granularity - distance)
        else:
            coord -= distance

        # Don't float if not needed
        if self.granularity >= 1.0:
            coord = int(coord)
        
        return coord


    def set_origin(self, origin_dict):
        '''
        Update the OccupancyGrid's origin position and quaternion

        Arguments:
        origin_dict  --  Dictionary containing new values for origin, with keys from 
                         ['x', 'y', 'z', 'quat_x', 'quat_y', 'quat_z', 'quat_w']
        '''
        new_ks = set(origin_dict.keys())
        ks = set(self.origin.keys())

        if new_ks.issubset(ks):
            self.origin.update(origin_dict)
            self.r.hmset(self.mapmeta, self.origin)
        else:
            raise KeyError("Expect subset or all of keys " + str(self.origin.keys()))



    def update(self, x, y, occupancy=100):
        '''
        Update an existing square in the map or add a new one.

        Arguments:
        x          --  x coordinate
        y          --  y coordinate
        occupancy  --  Occupancy certainty, -1 for unknown or [0..100]

        BE AWARE this snaps to the grid, 1,1 and 2,2 are not necessarily
        distinct squaresm depending on the granularity
        '''

        # Bump grid size if this is outside current bounding box
        self._bounds_check(x,y,True)

        k = self._genkey(x,y)
        if self.r.exists(k):
            existing = self.r.hgetall(k)
            # ...TODO - Update, decay, whatever
        self.r.hmset(k, {
            'occ'  : int(occupancy),
            'seen' : time.time()
        })
        if self.DEBUG:
            print("update {} {} to {}".format(x,y,occupancy))
        self.r.sadd(self.mapname, k)
        self.r.publish(self.channel, k)




    def touch(self, x, y):
        '''
        Set 'seen' key for a gridsquare to current time (a la *NIX 'touch')
        Returns false if the square does not exist
        '''
        k = self._genkey(x,y)
        if self.r.exists(k):
            self.r.hset(k, 'seen', time.time())
            return True
        return False



    def get_seentime(self, x, y):
        '''
        Return a gridsquare's last seen time as time since the UNIX epoch
        '''
        return self.r.hget(self._genkey(x,y), 'seen')



    def load(self, f=None):
        '''
        Load in a map from a given file.
        Provides a simplified model, a space indicates a certainly free space,
        a 'X' indicates a certainly occupied space and a ? indicates uncertainty.

        Arguments:
        f  --  The name of the file to parse; If none, uses built-in development string
        '''

        print("Loading...")

        if f is None:
            # Development & Testing map. Craxy indexing transforms into correct quadrant
            f = self._testworld.splitlines()[::-1]
        else:
            f = open(f, 'r')
            f = f.read().splitlines()

        for i, line in enumerate(f):
            print("")
            for j, char in enumerate(line):
                certainty = 0 # Default Unoccupied
                if char == 'X':
                    certainty = 100 # Occupied
                elif char == '?':
                    certainty = -1  # Unknown

                if self.DEBUG:
                    # Spew map at user
                    print(" {}({},{})".format(char,i* self.granularity,j* self.granularity), end='')

                # Multiply by granularity to scale onto native grid resolution
                self.update(float(j) * self.granularity,
                            float(i) * self.granularity,
                            certainty)
        
        if type(f) is file:
            f.close()
        
        print("")
        print("Done.")



    def dump(self, f=None):
        '''
        Dump map to a string or file

        TODO: NOTE: Not yet compatible with load()
        '''
        dump = yaml.dump(self.get_map())
        
        if f is not None:
            f = open(f, 'w')
            f.write(dump)
            f.close()

        return dump



    @requireimage
    def render(self, name=None):
        '''
        Render a bitmap image of the map in Greyscale + Alpha

        Arguments:
        name  --  Image name to save to, will not save if None

        Returns:
        PIL Image instance. Save with .save(), .show() is a preview
        '''
        w, h = self._get_map_dimensions()
        if self.DEBUG:
            print("Image dimensions are {}x by {}y".format(w, h))

        data = np.zeros((h, w, 4), dtype=np.uint8)
        
        map_data = self.get_map()
        
        for col in xrange(h-1):
            for row in xrange(w-1):
                occupancy = map_data[col][row]
                if occupancy >= 0:
                    # Greyscale
                    occupancy = int(2.55 * (100 - occupancy))
                    data[col, row] = [occupancy]*3 + [255]
                else:
                    # Unknown
                    data[col, row] = [0,255,0,0] # 100% transparent green
        
        img = PIL.Image.fromarray(data, 'RGBA')
        #i = PIL.Image.new(mode='LA', size=(width, height), color=0)

        if name is not None:
            img.save(name)
            print("Saved image as '{}'".format(name))

        return img



    def _genkey(self, x, y):
        '''
        Standardised generator for a hash key for a given coord.
        IMPORTANT: This needs to remain consistent with the self._dekey_re RegEx

        Returns:
        key  --  String with the global key name and x, y coords appended, snapped to the grid 
        '''
        return self.mapname + ":" + str(self._snap(x)) + ":" + str(self._snap(y))




    def _dekey(self, key):
        '''
        Turn a string into a (x,y) pair using the member regex _dekey_re
        and return said tuple
        '''
        try:
            return tuple(map(float, self._dekey_re.findall(key)[0]))
        except Exception as e:
            raise KeyError("Could not de-key {}  --  '{}'".format(key, e))



    def _bounds_check(self,x,y,expand=False):
        '''
        Assert (x,y) is within grid; If they don't and expand is True, 
        embiggen the grid to fit them.

        Arguments:
        x       --  X coordinate
        y       --  Y Coordinate
        expand  --  True automagics the map to be bigger, False doesn't

        Note this may become inconsistent with redis - this checks locally cached bounds
        '''
        if not expand:
            if x > self.bounds['maxx']:
                return False
            elif x < self.bounds['minx']:
                return False
            elif y > self.bounds['maxy']:
                return False
            elif y < self.bounds['miny']:
                return False

        if expand:
            change = False
            if x > self.bounds['maxx']:
                self.bounds['maxx'] = self._snap(float(x))
                change = True
            if x < self.bounds['minx']:
                self.bounds['minx'] = self._snap(float(x))
                change = True

            if y > self.bounds['maxy']:
                self.bounds['maxy'] = self._snap(float(y))
                change = True
            if y < self.bounds['miny']:
                self.bounds['miny'] = self._snap(float(y))
                change = True

            if change:
                self._meta_sync()
                # ^^ MUST remain consistent with DataStore.rospipe() processing
                if self.DEBUG:
                    print("Bounds-check increased dimensions {}".format(self.bounds))

        return True


    def _meta_sync(self):
        '''
        Re-sync map meta with Redis. The larger will persist, given there is
        no way to sync on timestamps
        '''
        redis_meta = self.r.hgetall(self.mapmeta)
        push_back = False

        for key, val in self.bounds.iteritems():
            try:
                if abs(float(redis_meta[key])) > abs(float(val)):
                    if self.DEBUG:
                        print("key '{}' value '{}' was bigger in Redis than cached '{}'"
                              "".format(key, redis_meta[key], val))
                    self.bounds[key] = redis_meta[key] # case redis was bigger
                    push_back = True
                    
                elif abs(float(redis_meta[key])) < abs(float(val)):
                    self.r.hset(self.mapmeta, key, val) # case we're bigger, push back
                    push_back = True
                
                # if same do nothing, we're fine
            except KeyError as e:
                print(e)
        
        if push_back:
            self.r.publish(self.channel, "#bounds {}".format(yaml.dump(self.bounds)))



    def _destroy(self):
        '''
        Delete the known world! (Interactive)
        '''
        if self.wt.confirm("Do you actually want to delete the map?\n\nThis is not undoable!",
                           default='no'):
        
            points = self.r.smembers(self.mapname)
            
            for key in points:
                self.r.delete(key)
                
            self.r.delete(self.mapname)
            self.r.delete(self.mapmeta)

            print("!!! Deleted the world from Redis !!!")
        
            # Hack but effective
            self.__init__(self.r, self.granularity, self.DEBUG)
            

        else:
            print("Did not delete the world")




#       ^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^



@requireros
class ROSGenerator:

    '''
    Assistant class, generates ROS Classes to be published to a 
    topic from data hashmap. USed by DataStore's rospipe()

    Prime candidate for the "Looks the most like Java Eww" award
    '''

    def __init__(self):
        '''
        All we have to do here is init a node
        '''
        rospy.init_node('talker', anonymous=True)
        
    
    def gen_pose(self, data, quat):
        pose = PoseStamped()

        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        
        pose.pose.position.x = float(data['x'])
        pose.pose.position.y = float(data['y'])
        pose.pose.position.z = 0.0
        
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()

        return pose


    def gen_odom(self, data, quat):
        odom = Odometry()
        opose = PoseWithCovariance()
        
        opose.pose.orientation.x = quat[0]
        opose.pose.orientation.y = quat[1]
        opose.pose.orientation.z = quat[2]
        opose.pose.orientation.w = quat[3]
        
        opose.pose.position.x = float(data['x'])
        opose.pose.position.y = float(data['y'])
        opose.pose.position.z = 0.0
        
        odom.header.frame_id = "map"
        odom.header.stamp = rospy.Time.now()
        odom.pose = opose

        return odom


    def gen_dist(self, data):
        # Publish a point cloud derived from the IR sensor activations
        dist = PointCloud()
        
        dist.header.frame_id = "khepera" # Tie to Khepera's frame of reference
        dist.header.stamp = rospy.Time.now()

        # Angles of the sensor from the X axis (in rad)
        sensor_angles = [
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
        sensor_offsets = [
            ( 15.0,  25.0),  # Left perpendicular
            ( 20.0,  20.0),  # Left angled
            ( 27.0,   8.0),  # Left forward
            ( 27.0,  -8.0),  # Right forward
            ( 20.0, -20.0),  # Right angled
            ( 15.0, -25.0),  # Right perpendicular
            (-26.0, -10.0),  # Back right
            (-26.0,  10.0)   # Back left
        ]

        keys = ['r0','r1','r2','r3','r4','r5','r6','r7']
        for key in keys:
            if key not in data.keys():
                raise KeyError("No range data -- Missing key " + 
                               str(key) + " " + str(data))

        pre_points = zip(keys, sensor_angles, sensor_offsets)
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
            
            pt = Point32()

            # point[2] is the sensor's coords relative to the robot
            # point[1] is the angle the sensor takes relative to the robot's x axis

            pt.x = (distance * math.cos(point[1])) + point[2][0]
            pt.y = (distance * math.sin(point[1])) + point[2][1]
            pt.z = 0.0

            intensities.append(distance)
            points.append(pt)

        dist.points = points
        intensities_chan = ChannelFloat32()
        intensity_chan = ChannelFloat32()

        intensities_chan.name = "intensities"
        intensities_chan.values = intensities
        intensity_chan.name = "intensity"
        intensity_chan.values = [
            30.0,   # min intensity
            1.0,    # max intensity
            0.0,    # min color
            1.0     # max color
        ]

        dist.channels = [intensities_chan, intensity_chan]
        
        return dist


    def gen_path(self):
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = "map"
        
        return path


    def gen_map(self, og):
        '''
        Arguments:
        og    --  Instance of a GridManager class
        '''
        m = OccupancyGrid()

        m.header.stamp = rospy.Time.now()
        m.header.frame_id = "map"

        # Width & height are a number of cells
        # Width folows x (forward) , height y
        m.info.width, m.info.height = og._get_map_dimensions()
        #m.info.width  = (-og.bounds['minx'] + og.bounds['maxx'] +1) / og.granularity
        #m.info.height = (-og.bounds['miny'] + og.bounds['maxy'] +1) / og.granularity
        # Units (metres, as far as ROS cares, but not in our case)
        m.info.resolution = og.granularity

        # Pose of the point zeroth datapoint, offset by half a granularity to reflect 
        # the behaviour of snapping
        m.info.origin.position.x = og.bounds['minx'] - og.origin['x'] - (float(og.granularity)/2.0)
        m.info.origin.position.y = og.bounds['miny'] - og.origin['y'] - (float(og.granularity)/2.0)
        m.info.origin.position.z = og.origin['z']
        m.info.origin.orientation.x = og.origin['quat_x']
        m.info.origin.orientation.y = og.origin['quat_y']
        m.info.origin.orientation.z = og.origin['quat_z']
        m.info.origin.orientation.w = og.origin['quat_w']
        data = og.get_map('N')

        m.data = data.flatten().tolist()

        return m



#       ^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^




# Only run this below code if we're invoked directly:
if __name__ == "__main__":

    args = sys.argv[1:]

    try:
        optlist, args = getopt.getopt(args, 
                                      'ds:prem:l:', 
                                      ['delete',
                                       'server=', 
                                       'plot', 
                                       'rospipe', 
                                       'replay', 
                                       'speed=',
                                       'load='])
    except getopt.GetoptError:
        print("Invalid Option, correct usage:")
        print("-d or --delete   : Destroy all data held in Redis")
        print("-s or --server   : Hostname of redis server to use. Default localhost")
        print("-p or --plot     : Live plot of published data")
        print("-r or --rospipe  : Pipe redis messages into ROS topics")
        print("-e or --replay   : Take historical data from Redis and re-publish to channel")
        print("-m or --speed    :     - Speed multiplier, default 1.0")
        print("-l or --load     : Load a map (occupancy grid) from file")
        sys.exit(2)

    server = "localhost"

    # Pre-instantiation options
    for opt, arg in optlist:
        if opt in ('-s', '--server'):
            print("Using Redis server at '" + str(arg) + "'")
            server = str(arg)

    ds = DataStore(host=server)
    speed = 1.0 # For replay 

    # Post-instantioation options
    for opt, arg in optlist:
        if opt in ('-d', '--delete'):
            ds._destroy()

        elif opt in ('-p', '--plot'):
            ds.plot()

        elif opt in ('-r', '--rospipe'):
            ds.rospipe()

        
        elif opt in ('-m', '--speed'):
            speed = float(arg)

        elif opt in ('-e', '--replay'):
            ds.replay(speed=speed)

        elif opt in ('-l', '--load'):
            ds.og.load(str(arg))

