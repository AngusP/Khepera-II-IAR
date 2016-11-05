#!/usr/bin/env/python

#
#     - - - -  I A R  - - - -
#
#  s1311631         Angus Pearson
#  s1346981    Jevgenij Zubovskij
#

from __future__ import print_function
from data import DataStore
import sys
import getopt


class Mapping(object):

    def __init__(self, host='localhost'):
        
        self.ds = DataStore(host=server)


    def sub_mapgen(self):
        '''
        Subscribe (so run async in a different process) to state updates published
        to Redis and affect the map
        '''
        sub = self.ds.r.pubsub()
        sub.subscribe([self.ds.listname])

        keys = set(['x','y','theta','t', 'r0','r1',
                    'r2','r3','r4','r5','r6','r7'])

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

                    print(data)
                    
            except KeyError as e:
                print("!!!!! EXCEPTION - Continuing. {}".format(str(e)))
                
                


    def _ir_to_dist(self, reading):
        '''
        From solved equation:
        y = 1.074519 + (10.57748 - 1.074519)/(1 + ( x /70.42612)^69.9039)^0.02119919
        '''
        return 10.0 * ( 1.074519 + (10.57748 - 1.074519)
                        /
                        math.pow(1 + (math.pow((reading / 70.42612),69.9039)), 0.02119919 ))



if __name__ == "__main__":


    args = sys.argv[1:]

    try:
        optlist, args = getopt.getopt(args, 's:', ['server='])
    except getopt.GetoptError:
        print("Invalid Option, correct usage:")
        print("-s or --server   : Hostname of redis server to use. Default localhost")
        sys.exit(2)

    server = "localhost"

    # Pre-instantiation options
    for opt, arg in optlist:
        if opt in ('-s', '--server'):
            print("Using Redis server at '" + str(arg) + "'")
            server = str(arg)

    m = Mapping()


