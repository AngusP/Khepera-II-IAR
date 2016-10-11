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

class DataStore:

    def __init__(self, host='localhost', db=0, port=6379):
        self.r = redis.StrictRedis(host=host, port=port, db=db)
        self.wt = whiptail.Whiptail()
        self.listname = "statestream"

        self.r.ping()

    def __del__(self):
        self.save()

    def keys(self):
        return self.r.keys()

    def push(self, point):
        # We only accept a specific data type:
        if not isinstance(point, GenericState):
            raise TypeError("Expected instance of GenericState or derivative in DataStore.push")

        # Build hashmap from given state class
        hmap = {
            't'    : point.time,
            'x'    : point.x,
            'y'    : point.y,
            'theta': point.theta
        }
        
        # For each part of the _python_ dict we
        # create a Redis hashmap using the time as index
        for key, value in hmap.iteritems():
            self.r.hset(point.time, key, value)

        # We push a reference to this new hashmap onto 
        # the statestream list
        self.r.lpush(self.listname, point.time)

        # Also publish onto a channel
        self.r.publish(self.listname, point.time)
        # Decrememt reference counter
        #del point

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

    def _purge(self):
        if self.wt.confirm("Really destroy all data in Redis store?\n\nThis is not undoable!\n(run FLUSHDB)",
                           default='no'):
            self.r.flushdb()
            print("!!! Flushed Redis Data Store !!!")
        else:
            print("Did not purge Redis Store")

    def plot(self):
        print("Plotting Subroutine...")
        try:
            pubsub = self.r.pubsub()
            pubsub.subscribe([self.listname])
            #plt.axis([-500, 500, -500, 500])
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


    def _pubtest(self):
        print("PubSub test")
        # Publish cosine to the channel
        data = {
            'x' : 0.0,
            'y' : 0.0
        }

        try:
            while True:
                data['x'] = data['x'] + 0.01
                data['y'] = math.cos(data['x'])
                self.r.hset('testh', 'x', data['x'])
                self.r.hset('testh', 'y', data['y'])
                self.r.publish(self.listname, 'testh')
                time.sleep(0.05)
        except KeyboardInterrupt as e:
            self.r.delete('testh')
            print(e)


# Only run if we're invoked directly:
if __name__ == "__main__":

    args = sys.argv[1:]

    try:
        optlist, args = getopt.getopt(args, 'dts:p', ['delete','test','server=', 'plot'])
    except getopt.GetoptError:
        print("Invalid Option, correct usage:")
        print("-d or --delete : Destroy all data held in Redis")
        print("-t or --test   : Publish test data to a plotter")
        print("-s or --server : Hostname of redis server to use. Default localhost")
        print("-p or --plot   : Live plot of published data")
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
        
        elif opt in ('-t', '--test'):
            ds._pubtest()

        elif opt in ('-p', '--plot'):
            ds.plot()

