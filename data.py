from __future__ import print_function
import redis
import whiptail
from state import *
import sys
import getopt

class DataStore:

    def __init__(self, host='localhost', db=0, port=6379):
        self.r = redis.StrictRedis(host=host, port=port, db=db)
        self.wt = whiptail.Whiptail()
        self.listname = "statestream"

    def __del__(self):
        self.save()

    def push(self, point):
        # We only accept a specific data type:
        if not isinstance(point, GenericState):
            raise TypeError("Expected instance of GenericState or derivative")

        # Build hashmap from given state class
        hmap = {
            't'    : point.time,
            'x'    : point.x,
            'y'    : point.y,
            'theta': point.theta,
            'e_l'  : point.encoder_l,
            'e_r'  : point.encoder_r
        }
        
        # For each part of the _python_ dict we
        # create a Redis hashmap using the time as index
        for key, value in hmap.iteritems():
            self.r.hset(point.time, key, value)

        # We push a reference to this new hashmap onto 
        # the statestream list
        self.r.lpush(self.listname, point.time)
        # Decrememt reference counter
        del point

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
            ret[int(item['t'])] = item
            
        return ret

    def save(self):
        # Copy DB to disk
        return self.r.save()

    def delete_before(self, time):
        # Remove data with a key from earlier than specified

        if time < 0:
            raise ValueError("Time must be positive you crazy person!")

        keys = self.get(0, -1) # All keys
        
        print(keys)

        for key in keys:
            if int(key) <= int(time):
                self.r.lrem(self.listname, key)
                self.r.delete(key)

    def _purge(self):
        if self.wt.confirm("Really destroy all data in Redis store?\n\nThis is not undoable!\n(run FLUSHDB)",
                           default='no'):
            self.r.flushdb()
            print("!!! Flushed Redis Data Store !!!")
        else:
            print("Did not purge Redis Store")


# Only run if we're invoked directly:
if __name__ == "__main__":

    ds = DataStore()
    s1 = GenericState()
    
    args = sys.argv[1:]

    try:
        optlist, args = getopt.getopt(args, 'pt', ['purge','test'])
    except getopt.GetoptError:
        print("Invalid Option, correct usage:")
        print("-p or --purge : Destroy all data held in Redis")
        print("-t or --test  : Development tests")
        sys.exit(2)

    for opt, arg in optlist:
        if opt in ('-p', '--purge'):
            ds._purge()
        elif opt in ('-t', '--test'):
            ## TEST CODE HERE ##
            ds.push(s1)
            print(ds.get())
            ds.delete_before(10)

