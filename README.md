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




