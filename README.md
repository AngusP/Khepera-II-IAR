# IAR 2016 Source

### Authors:

s1311631
: Angus Pearson

s1346981
: Jevgenij Zubovskij

### Install/Setup

Use of a virtualenv is suggested, which can be set up using `virtualenv VENV`.
Virtualenv is then applied using `> source ./VENV/bin/activate` and deactivated using
`(VENV)> deactivate`.

Code targets Python 2.7, but makes use of some future features.

Requirements can be installed using `(VENV)> pip install -r ./requirements.txt`. 
Pip probably should be upgraded as well, using `(VENV)> pip install --upgrade pip`.

#### Redis

Download [Redis](http://redis.io) and follow it's instructions for compiling, testing and installing. A `redis-server` instance needs to be accessible for this code to run correctly, it does not have to be running on the same machine.

The server instance should be run with the given `redis.conf` file: 

    $ redis-server ./redis.conf
    
                    _._
               _.-``__ ''-._
          _.-``    `.  `_.  ''-._           Redis 3.2.4 (00000000/0) 64 bit
      .-`` .-```.  ```\/    _.,_ ''-._ 
     (    '      ,       .-`  | `,    )     Running in standalone mode
     |`-._`-...-` __...-.``-._|'` _.-'|     Port: 6379
     |    `-._   `._    /     _.-'    |     PID: 10826
      `-._    `-._  `-./  _.-'    _.-' 
     |`-._`-._    `-.__.-'    _.-'_.-'|
     |    `-._`-._        _.-'_.-'    |           http://redis.io
      `-._    `-._`-.__.-'_.-'    _.-'
     |`-._`-._    `-.__.-'    _.-'_.-'|
     |    `-._`-._        _.-'_.-'    |
      `-._    `-._`-.__.-'_.-'    _.-'
          `-._    `-.__.-'    _.-'
              `-._        _.-'
                  `-.__.-'


Bear in mind Redis is an in-memory data store, so on-disk saves won't occur that often; Manual saves may be neecded to force data retention.

### Plotting


    (VENV)$ python -i ./data.py
    >>> ds.plot()         # Live plotting
    >>> ds.static_plot()  # plot existing data
    >>> ds._purge()       # Clear Redis
    >>> ds.get()          # return all data


Further reference:

    (VENV)$ python ./data.py --help
    
Methods in `data.py` also have documentation

Command-line shortcuts are

    data.py --server=<REDIS SERVER>
    data.py -s <REDIS SERVER>
    
    data.py --delete
    data.py -d
    
    data.py --plot
    data.py -p
    
    data.py --rospipe
    data.py -r


# ROS Integration

Internally we're using Redis to publish and collect data. `data.py` has a method `DataStore.rospipe()` that pipes from Redis' PubSub into ROS topics. It can be run as a method call, or with the command line switch `--rospipe` or `-r`.

[ROS.org](http://ros.org) has detailed docs on installation and dependencies. For the whole ROS pipeline to work, and instance of `roscore` must be running locally, and `rospipe()` will publish to the topics `/statestreampose`, `/statestreamodom` and `/statestreamdist`, as well as publishing a `TR` named `khepera`. Rviz should be able to detect these topics.


# Utilities (utils.py)

**Reference Frame Transform**

    utils.relative_to_fixed_frame_tf(fixed_x, fixed_y, relative_theta, relative_x, relative_y)

**IR raw sensor reading to distance**

    utils.ir_to_dist(reading)



# Occupancy Grid

A `DataStore` instance has it's own instance of a `GridManager`, called `og`. 


## Generating a map

The mapper is run asynchronously from other programs, like so:

    $ python mapping.py --map

## Main Methods and Members

**NB:**

Full class names are given, though these methods and members should be accessed 
though an instance, like so:

    >>> ds = DataStore()
    >>> ds.push(...)
    >>> ds.og.get(...) # 'og' is a DataStore's GridManager instance 

**DocStrings** should be consulted for fuller documentation

GridManager.get(x, y)
: Gives occupancy - unoccupied to occupied [0..100], -1 unknown

GridManager.mget(l)
: Gives list of occupancies

GridManager.granularity
: Stores the granular unit size for the Occupancy grid.

GridManager._snap(coord)
: takes an arbitrarily accurate number and returns the coord of the grid cell it falls within. You probably shouldn't use this (hence the underscore), as `.get()` automatically calls this on it's args.


## Deleting data & cleaning up

The occupancy grid can be destroyed (keeping all other data such as poses) using the `_destroy()` method,
e.g. `ds.og._destroy()`. This **is different** from the `DataStore._destroy()` method that will 
flush the *entire* database.

The destroy method can be called directly from the command line:

    (VENV) $ python mapping.py --destroy

All listeners and programs using an occupancy grid should be restarted after a destroy.


