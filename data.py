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


# Check for ROS (http://ros.org/) support
try:
    import rospy
    ros = True
    from std_msgs.msg import String
    from geometry_msgs.msg import PoseWithCovariance, PoseStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Range
    import tf
except ImportError as e:
    print(e)
    print("Continuing without ROS integration")
    ros = False
    pass



class DataStore:

    def __init__(self, host='localhost', db=0, port=6379):
        self.r = redis.StrictRedis(host=host, port=port, db=db)
        self.wt = whiptail.Whiptail()
        # Redis List and Channel name
        self.listname = "statestream"
        # ROS Topics
        self.posetopic = self.listname + "pose"
        self.odomtopic = self.listname + "odom"

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
        print("Plotting Subroutine...")
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

            
    def static_plot(self):
        # Plot all existing data after a run
        data = self.get()
        
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

        pose_pub = rospy.Publisher(self.posetopic, PoseStamped, queue_size=10)
        odom_pub = rospy.Publisher(self.odomtopic, Odometry, queue_size=10)
        rospy.init_node('talker', anonymous=True)

        sub = self.r.pubsub()
        sub.subscribe([self.listname])

        # Loop until stopped plotting the path
        for item in sub.listen():

            if rospy.is_shutdown():
                print("\nStopping rospipe...")
                break
            
            if self.r.hexists(item['data'], 'x'):
                # Pull from redis:
                data = self.r.hgetall(item['data'])
                # Generate new pose
                pose = PoseStamped()
                
                quat = tf.transformations.quaternion_from_euler(0.0, 0.0, float(data['theta']))
                pose.pose.orientation.x = quat[0]
                pose.pose.orientation.y = quat[1]
                pose.pose.orientation.z = quat[2]
                pose.pose.orientation.w = quat[3]
                
                pose.pose.position.x = float(data['x'])
                pose.pose.position.y = float(data['y'])
                pose.pose.position.z = 0.0
                
                pose.header.frame_id = "map"
                pose.header.stamp = rospy.Time.now()

                # Generate odometry data
                odom = Odometry()
                opose = PoseWithCovariance()
                
                odom.header.frame_id = "map"
                odom.header.stamp = rospy.Time.now()
                opose.pose.position.x = float(data['x'])
                opose.pose.position.y = float(data['y'])
                opose.pose.position.z = 0.0
                opose.pose.orientation.x = quat[0]
                opose.pose.orientation.y = quat[1]
                opose.pose.orientation.z = quat[2]
                opose.pose.orientation.w = quat[3]
                odom.pose = opose
                
                # rospy.loginfo(pose)
                # Publish to ROS topic
                pose_pub.publish(pose)
                odom_pub.publish(odom)

            
    def _purge(self):
        # Clear out all (literally all) data held in Redis
        if self.wt.confirm("Really destroy all data in Redis store?\n\nThis is not undoable!\n(run FLUSHDB)",
                           default='no'):
            self.r.flushdb()
            print("!!! Flushed Redis Data Store !!!")
        else:
            print("Did not purge Redis Store")

            
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

    # Post-instantioation options
    for opt, arg in optlist:
        if opt in ('-d', '--delete'):
            ds._purge()

        elif opt in ('-p', '--plot'):
            ds.plot()

        elif opt in ('-r', '--rospipe'):
            ds.rospipe()

