#!/usr/bin/env/python

#
#     - - - -  I A R  - - - -
#
#  s1311631         Angus Pearson
#  s1346981    Jevgenij Zubovskij
#

from data import DataStore

class Mapping(object):

    def __init__(self, host='localhost'):
        
        self.ds = DataStore(host=server)

        

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

    


