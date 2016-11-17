#
#     - - - -  I A R  - - - -
#
#  s1311631         Angus Pearson
#  s1346981    Jevgenij Zubovskij
#

#REACTIVE THRESHOLDS

CONST_SPEED = 4
CONST_WALL_DIST = 200

#TURN SCALING

TURN_LESS = 0.2
TURN_MORE = 1.0

#REACTIVE STATES

STATE_DRIVE_FORWARD = 0
STATE_DRIVE_BACKWARD = 1
STATE_STUCK_LEFT = 2
STATE_STUCK_RIGHT = 3
STATE_LEFT_FOLLOW = 4
STATE_RIGHT_FOLLOW   = 5
STATE_BOREDOM_ROTATE = 6
STATE_BOREDOM_DRIVE  = 7

#ENCODER_CONSTANTS

TICKS_PER_MM = 12.0    # encoder ticks per milimeter
TICKS_PER_M = TICKS_PER_MM * 1000.0 # encoder ticks per meter

# PHYSICAL CONSTANTS

WHEEL_BASE_MM = 56.0 # mm
WHEEL_BASE_M = WHEEL_BASE_MM / 1000.0 # m


#period for waiting when grabbing / uncloading food
WAIT_PERIOD_S = 2

#wait period to wait before sending new serial stuff
MEASUREMENT_PERIOD_S = 0.1 # s
MEASUREMENT_PERIOD_MS = int(MEASUREMENT_PERIOD_S * 1000) # s


# SENSOR CONSTANTS

DIST_CUTOFF = 0.0

#pathing stuff

#Angle threshold until we begin correcting out orientation along the M-line
M_N_ANGLE  = 10 # degrees

#maximum boredom rounds the robot can do
MAX_BOREDOM = 10

#coeffiecient as to how many round it takes to form a spiral inceremnt speed
SPIRAL_SPEED_COEFFICIENT = 10




