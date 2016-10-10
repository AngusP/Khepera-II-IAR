from __future__ import print_function
import redis
import whiptail
from state import *
import sys
import getopt

class DataStore:

    def __init__(self, host='localhost', db=9, port=6379):
        self.r = redis.StrictRedis(host=host, port=port, db=db)
        self.wt = whiptail.Whiptail()
        self.listname = "statestream"

    def push(self, this):
        # We only accept a specific data type:
        if not isinstance(this, GenericState):
            raise TypeError

        todo = {"key1":1, "key2":2}
        
        # For each part of the _python_ dict we
        # create a Redis hashmap using the time as index
        for key, value in todo.iteritems():
            self.r.hset(this.time, key, value)
    
        # We push a reference to this new hashmap onto 
        # the statestream list
        self.r.lpush(self.listname, this.time)
        # Decrememt reference counter
        del this

    def get(self, start, stop):
        # Get references from list
        keys = self.r.lrange(self.listname, int(start), int(stop))
        ret = dict()

        for key in keys:
            ret[int(key)] = self.r.hgetall(key)

        return ret

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
        optlist, args = getopt.getopt(args, 'p', ['purge','test'])
    except getopt.GetoptError:
        print("Invalid Option, correct usage:")
        print("-p or --purge : Destroy all data held in Redis")
        print("-t or --test  : Development tests")
        sys.exit(2)

    for opt, arg in optlist:
        if opt in ('-p', '--purge'):
            ds._purge()
        elif opt in ('-t', '--test'):
            ds.push(s1)

