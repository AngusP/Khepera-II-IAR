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

import constants


# Check for ROS (http://ros.org/) support
try:
    import rospy
    ros = True
    from std_msgs.msg import String
    from geometry_msgs.msg import PoseWithCovariance, PoseStamped, Point32
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud, ChannelFloat32
    import tf
except ImportError as e:
    print(e)
    print("Continuing without ROS integration")
    ros = False
    pass




#       ^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^



class DataStore:

    def __init__(self, host='localhost', db=0, port=6379):
        self.r = redis.StrictRedis(host=host, port=port, db=db)
        self.wt = whiptail.Whiptail()
        # Redis List and Channel name
        self.listname = "statestream"
        # ROS Topics
        self.posetopic = self.listname + "pose"
        self.odomtopic = self.listname + "odom"
        self.disttopic = self.listname + "dist"

        self.r.ping()

    def __del__(self):
        self.save()

    def keys(self):
        return self.r.keys()

    
    def push(self, pose, ranges=None):
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
        self.r.lpush(self.listname, mapname)

        # Also publish onto a channel
        self.r.publish(self.listname, mapname)

        
    def get(self, start=0, stop=-1):
        # Returns a list in-order over the range
        keys = self.r.lrange(self.listname, int(start), int(stop))
        ret = list()

        # Pull all the hashmaps out of the store
        for key in keys:
            ret.append(self.r.hgetall(key))

        return ret

    
    def get_dict(self, start=0, stop=-1):
        # Return a dictionary instead of a list
        lst = self.get(start, stop)
        ret = dict()
        for item in lst:
            # time as key, dict as value
            ret[int(item['t'])] = item
            
        return ret

    
    def save(self):
        # Copy DB to disk
        return self.r.save()

    
    def delete_before(self, time):
        # Remove data with a key from earlier than specified

        if time < 0:
            raise ValueError("Time must be positive you crazy person!")

        keys = self.r.lrange(self.listname, 0, -1) # All keys in our stream
        
        for key in keys:
            if int(key) <= int(time):
                # Remove from list
                self.r.lrem(self.listname, count=0, value=key)
                # delete the key (hashmap)
                self.r.delete(key)

    def plot(self):
        self.static_plot()

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
        # Plot all existing data after a run
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


    def rospipe(self):
        print("Piping Redis ---> ROS")
        # Pipe data out of Redis into ROS

        if not ros:
            raise ImportError("ROS (rospy) Not supported/found")

        rg = ROSGenerator()
        
        pose_pub = rospy.Publisher(self.posetopic, PoseStamped, queue_size=10)
        odom_pub = rospy.Publisher(self.odomtopic, Odometry, queue_size=10)
        dist_pub = rospy.Publisher(self.disttopic, PointCloud, queue_size=10)
        tbr = tf.TransformBroadcaster()
        
        rospy.init_node('talker', anonymous=True)

        sub = self.r.pubsub()
        sub.subscribe([self.listname])

        # Loop until stopped plotting the path
        for item in sub.listen():

            if rospy.is_shutdown():
                print("\nStopping rospipe...")
                break

            if item['type'] == "subscribe":
                rospy.loginfo("Subscribed successfully")
                continue
            
            try:
                # Pull from redis:
                data = self.r.hgetall(item['data'])
                
                required_keys = ['x','y','theta','t']
                for key in required_keys:
                    if key not in data.keys():
                        raise KeyError("Missing key " + str(key) + " from hashmap " + str(data))

                # Generate a new Quaternion based on the robot's pose
                quat = tf.transformations.quaternion_from_euler(0.0, 0.0, float(data['theta']))
            
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

                rospy.loginfo(" Redis --> ROS #" + str(round(float(data['t']), 4)))

            except KeyError as e:
                rospy.logwarn(str(e))
                pass


    def _purge(self):
        # Clear out all (literally all) data held in Redis
        if self.wt.confirm("Really destroy all data in Redis store?\n\nThis is not undoable!\n(run FLUSHDB)",
                           default='no'):
            self.r.flushdb()
            print("!!! Flushed Redis Data Store !!!")
        else:
            print("Did not purge Redis Store")



#       ^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^



# Assistant class, generates ROS Classes from data hashmap
class ROSGenerator:

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
        
        dist.header.frame_id = "map"
        dist.header.stamp = rospy.Time.now()

        robot_pos = (float(data['x']), float(data['y']))
        robot_ang = float(data['theta'])

        # Angles of the sensor from the X axis (in rad)
        sensor_angles = [
            0.5 * math.pi,  # Left perpendicular
            0.25 * math.pi, # Left angled
            0.0,            # Left forward
            0.0,            # Right forward
            1.75 * math.pi, # Right angled
            1.5 * math.pi,  # Right perpendicular
            math.pi,        # Back right
            math.pi         # Back left
        ]

        # Physical (x,y) offset of the sensor from the center of the bot in mm
        sensor_offsets = [
            (-25.0, 15.0),  # Left perpendicular
            (-20.0, 20.0),  # Left angled
            (-8.0,  27.0),  # Left forward
            (8.0,   27.0),  # Right forward
            (20.0,  20.0),  # Right angled
            (25.0,  15.0),  # Right perpendicular
            (-10.0, -26.0), # Back right
            (10.0, -26.0),  # Back left
        ]

        keys = ['r0','r1','r2','r3','r4','r5','r6','r7']
        for key in keys:
            if key not in data.keys():
                raise KeyError("No range data -- Missing key " + str(key) + " " + str(data))

        pre_points = zip(keys, sensor_angles, sensor_offsets)
        points = []
        intensities = []

        # Basic trig, converts ranges to points relative to robot
        for point in pre_points:
            reading = float(data[point[0]])
            intensities.append(reading)
            
            pt = Point32()
            #                         Offset from center of body v
            pt.x = reading * math.cos(point[1] + robot_ang) + robot_pos[0] + point[2][0]
            pt.y = reading * math.sin(point[1] + robot_ang) + robot_pos[1] + point[2][0]
            pt.z = 16.0 # Sensors are 16mm off ground

            points.append(pt)

        dist.points = points
        intensities_chan = ChannelFloat32()
        intensity_chan = ChannelFloat32()

        intensities_chan.name = "intensities"
        intensities_chan.values = intensities
        intensity_chan.name = "intensity"
        intensity_chan.values = [
            50.0,   # min intensity
            1000.0, # max intensity
            0.0,    # min color
            1.0     # max color
        ]

        dist.channels = [intensities_chan, intensity_chan]
        
        return dist
        



#       ^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^v^




# Only run if we're invoked directly:
if __name__ == "__main__":

    args = sys.argv[1:]

    try:
        optlist, args = getopt.getopt(args, 'ds:pr', ['delete','server=', 'plot', 'rospipe'])
    except getopt.GetoptError:
        print("Invalid Option, correct usage:")
        print("-d or --delete   : Destroy all data held in Redis")
        print("-s or --server   : Hostname of redis server to use. Default localhost")
        print("-p or --plot     : Live plot of published data")
        print("-r or --rospipe  : Pipe redis messages into ROS topics")
        sys.exit(2)

    server = "localhost"

    # Pre-instantiation options
    for opt, arg in optlist:
        if opt in ('-s', '--server'):
            print("Using Redis server at '" + str(arg) + "'")
            server = str(arg)

    ds = DataStore(host=server)

    # Random convenience vars
    pos = {
        'x': 0,
        'y': 0,
        't':0,
        'theta':0.0,
        'r0': 10,
        'r1': 10,
        'r2': 10,
        'r3': 10,
        'r4': 10,
        'r5': 10,
        'r6': 10,
        'r7': 10
    }
    ran = [0,1,2,3,4,5,6,7]

    # Post-instantioation options
    for opt, arg in optlist:
        if opt in ('-d', '--delete'):
            ds._purge()

        elif opt in ('-p', '--plot'):
            ds.plot()

        elif opt in ('-r', '--rospipe'):
            ds.rospipe()

