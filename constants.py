#
#     - - - -  I A R  - - - -
#
#  s1311631         Angus Pearson
#  s1346981    Jevgenij Zubovskij
#

#REACTIVE THRESHOLDS

CONST_SPEED = 8
CONST_WALL_DIST = 200

#BOREDOM CONSTANTS

CONST_WALL_BORED_MAX = 100
CONST_BORED_TURN_MAX = 20

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

#called 180 because we were going away from the goal and now are turning towards it
STATE_BUG_180 = 8


#ENCODER_CONSTANTS

TICKS_PER_MM = 12.0    # encoder ticks per milimeter
TICKS_PER_M = TICKS_PER_MM * 1000.0 # encoder ticks per meter

# PHYSICAL CONSTANTS

WHEEL_BASE_MM = 56.0 # mm
WHEEL_BASE_M = WHEEL_BASE_MM / 1000.0 # m



MEASUREMENT_PERIOD_S = 0.05 # s

#Not exactly as expected as 30 seconds is not just delay, but also computations, hence value below is 15, not 30
EXPLORATION_CYCLES = 50.0 / MEASUREMENT_PERIOD_S 


# SENSOR CONSTANTS

DIST_CUTOFF = 0.0

#Unscaled distance gradient at which we should recompute the M-line
M_DISTANCE = 10 # mm

#Angle threshold until we begin correcting out orientation along the M-line
M_N_ANGLE  = 10 # degrees

#Distance at which we still consider ourselves to be following the M-line
ON_MLINE   = 30 #mm


