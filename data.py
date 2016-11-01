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


# Check for ROS (http://ros.org/) support
try:
    import rospy
    ros = True
    from std_msgs.msg import String
    from geometry_msgs.msg import PoseWithCovariance, PoseStamped, Point32
    from nav_msgs.msg import Odometry, Path, OccupancyGrid, MapMetaData
    from sensor_msgs.msg import PointCloud, ChannelFloat32
    import tf
except ImportError as e:
    print(e)
    print("Continuing without ROS integration")
    ros = False
    pass

# ROS Import check wrapper
def requireros(func):
    def check_and_call(*args, **kwargs):
        if not ros:
            raise ImportError("ROS (rospy) Not supported/found")
        else:
            return func(*args, **kwargs)
    return check_and_call


#       ^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^



class DataStore:

    '''
    Primary Data Munging class, handles chatter with Redis and ROS.
    `data.py` can be used interactively.
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

        self.og = GridManager(self.r)

        # Redis List and Channel name
        self.listname = "statestream"
        self.goallist = "goalstream"
        # ROS Topics
        self.posetopic = self.listname + "pose"
        self.odomtopic = self.listname + "odom"
        self.disttopic = self.listname + "dist"
        self.goaltopic = self.listname + "goal"
        self.maptopic  = self.listname + "map"

        # Test Redis connection
        self.r.ping()

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
        
        rospy.init_node('talker', anonymous=True)

        sub = self.r.pubsub()
        sub.subscribe([self.listname, self.goallist])

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
                        

                else:
                    # If we get an unexpected channel, complain loudly
                    raise ValueError("Encountered unknown channel '" + 
                                     str(item['channel']) + "'")


            except (KeyError, ValueError) as e:
                rospy.logwarn(str(e))
                pass


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

  <<<------------------------###
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
    
    def __init__(self, redis, granularity=1.0):
        '''
        Arguments:
        granularity  --  Minimum distance representable in the map, as a decimal multiple of 
                         whatever units the distances are given in.
        '''
        self.granularity = granularity
        
        self.r = redis
        # Redis list and channel names
        self.mapname = "map"
        self.mapmeta = self.mapname + "-meta"
        self.channel = self.mapname + "-update"
        
        # Default origin
        self.origin = {
            'x'       : 0.0,
            'y'       : 0.0,
            'z'       : 0.0,
            'quat_x'  : 0.0,
            'quat_y'  : 0.0,
            'quat_z'  : 0.0,
            'quat_w'  : 0.0
        }

        self.bounds = {
            'maxx'  :  10.0,
            'maxy'  :  10.0,
            'minx'  : -10.0,
            'miny'  : -10.0
        }

        # If prior config exists, pull it in
        for k, v in self.origin.iteritems():
            if self.r.hexists(self.mapmeta, k):
                self.origin[k] = float(self.r.hget(self.mapmeta, k))


        for k, v in self.bounds.iteritems():
            if self.r.hexists(self.mapmeta, k):
                self.bounds[k] = float(self.r.hget(self.mapmeta, k))

        # Push origin and boundaries back to redis
        self.r.hmset(self.mapmeta, self.origin)
        self.r.hmset(self.mapmeta, self.bounds)


        # Regex to turn a serialised key into coords
        # This re finds a combination of two floats or two ints
        # IMPORTANT: Must remain consistent with _genkey()
        self._dekey_re = re.compile("([+-]?\d+\.\d*|\d+):([+-]?\d+\.\d*|\d+)$")
        
        self.wt = whiptail.Whiptail()
        self.pp = pprint.PrettyPrinter(indent=1)


    def __str__(self):
        '''
        return a string representation of self
        '''
        return self.pp.pformat(self.__dict__)

    __repr__ = __str__


    def get(self, x, y):
        '''
        Returns the int held at the given grid coord's occupancy value
        '''
        occ = self.r.hget(self._genkey(x,y), 'occ')
        if occ is None:
            return None
        else:
            return int(occ)




    def get_map(self):
        '''
        Returns the entire known map, in a ROS friendly format
        of a 2D array, with floating occupancies. Default value is -1
        '''
        xwidth = int(math.ceil(-self.bounds['minx'] + self.bounds['maxx']))
        ywidth = int(math.ceil(-self.bounds['miny'] + self.bounds['maxy']))

        # Initialise a 2D array of the correct size
        data = [-1]*ywidth
        for row in data:
            row = [-1]*xwidth
        
        pts_keys = self.r.hgetall(self.mapname)

        for pt_key, grid_key in pts.iteritems():
            grid_hm = self.r.hgetall(grid_key)
            x, y = self._dekey(pt_key)
            # Boundaries as per ROS spec
            data[x][y] = max(0, min(int(occupancy), 100))

        return data



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



    def update(self, x, y, occupancy):
        # remember to update bounds if outside
        # and to publish to an updated coords channel
        raise NotImplementedError()
        


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
            return map(float, self._dekey_re.findall(key)[0])
        except Exception as e:
            raise KeyError("Could not de-key " + str(key) + "  --  '" + str(e) + "'")


    def _snap(self, coord):
        '''
        Snap a given coordinate to the grid

        Arguments:
        coord  --  Arbuitary coordinate (float, int)

        Returns:
        coord  --  Snapped to grid. int or float, depending on whether the granularity is an
                   integer or float.
        '''
        # See how far it is from the nearest lower point
        distance = float(coord) % self.granularity
        
        # If closer to a higher one, push it up
        if distance >= self.granularity/2.0:
            coord += (self.granularity - distance)
        else:
            coord -= distance

        # if self.granularity < 1.0:
        #     # If the granularity is sub-integer, try and kill any floating point artefacts
        #     # Get order of magnitude:
        #     om = int(round(math.log10(self.granularity), 0))
        #     coord = round(coord, -om)
        # else:
        #     # If it isn't, ints make all floating point problems go away
        #     coord = int(coord)

        if self.granularity >= 1.0:
            coord = int(coord)
        
        return coord


    def _destroy(self):
        '''
        Delete the known world (Interactive)
        '''
        if self.wt.confirm("Do you actually want to delete the map?\n\nThis is not undoable!",
                           default='no'):
        
            points = self.r.hgetall(self.mapname)
            
            for key, val in points.iteritems():
                self.r.delete(key)
                
                self.r.delete(self.mapname)
                self.r.delete(self.mapmeta)
                
                
            print("!!! Deleted the world from Redis !!!")

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
            distance = self._ir_to_dist(reading)

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

    def gen_path_pose(self, data):
        pass


    def _ir_to_dist(self, reading):
        '''
        From solved equation:
        y = 1.074519 + (10.57748 - 1.074519)/(1 + ( x /70.42612)^69.9039)^0.02119919
        '''
        return 10.0 * ( 1.074519 + (10.57748 - 1.074519)
                        /
                        math.pow(1 + (math.pow((reading / 70.42612),69.9039)), 0.02119919 ))



#       ^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^




# Only run this below code if we're invoked directly:
if __name__ == "__main__":

    args = sys.argv[1:]

    try:
        optlist, args = getopt.getopt(args, 
                                      'ds:prem:', 
                                      ['delete','server=', 'plot', 'rospipe', 'replay', 'speed='])
    except getopt.GetoptError:
        print("Invalid Option, correct usage:")
        print("-d or --delete   : Destroy all data held in Redis")
        print("-s or --server   : Hostname of redis server to use. Default localhost")
        print("-p or --plot     : Live plot of published data")
        print("-r or --rospipe  : Pipe redis messages into ROS topics")
        print("-e or --replay   : Take historical data from Redis and re-publish to channel")
        print("-m or --speed    :     - Speed multiplier, default 1.0")
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

