import redis

r = redis.StrictRedis(host='localhost', port=6379, db=0)

class DataStore:

    def __init__(self):
        raise NotImplementedError
