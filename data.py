from __future__ import print_function
import redis
import whiptail
from state import *
import sys
import getopt

class DataStore:

    def __init__(self):
        self.r = redis.StrictRedis(host='localhost', port=6379, db=0)
        self.wt = whiptail.Whiptail()
        self.listname = "statestream"

    def push(self, this):
        if not isinstance(this, GenericState):
            raise TypeError

        todo = {"key1":1, "key2":2}
        
        for key, value in todo.iteritems():
            print(key)
            print(value)
            
        self.r.lpush(self.listname, todo)
        del this

    def get(self, start, stop):
        return self.r.lrange(self.listname, int(start), int(stop))

    def _purge(self):
        if self.wt.confirm("Really destroy all data in Redis store?\n\nThis is not undoable!\n(run FLUSHDB)",
                           default='no'):
            self.r.flushdb()
            print("!!! Flushed Redis Data Store !!!")
        else:
            print("Did not purge Redis Store")


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

