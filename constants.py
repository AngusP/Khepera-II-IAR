#
#     - - - -  I A R  - - - -
#
#  s1311631         Angus Pearson
#  s1346981    Jevgenij Zubovskij
#

#REACTIVE CONTROL CONSTANTS

CONST_SPEED = 8
CONST_WALL_DIST = 160
CONST_WALL_OFFSET = CONST_WALL_DIST * 0.2
CONST_INF_DIST = CONST_WALL_DIST * 0.5

CONST_WALL_BORED_MAX = 100000
CONST_BORED_TURN_MAX = 20
CONST_TURN_PROPORTION = 0.3

DIR_LEFT = 0
DIR_RIGHT = 1


STATE_DRIVE_FORWARD = 0
STATE_DRIVE_BACKWARD = 1
STATE_STUCK_LEFT = 2
STATE_STUCK_RIGHT = 3
STATE_LEFT_FOLLOW = 4
STATE_RIGHT_FOLLOW = 5
STATE_BOREDOM_ROTATE = 6
STATE_BOREDOM_DRIVE = 7


#ODOMETRY_CONSTANTS

TICKS_PER_MM = 12.0    # encoder ticks per milimeter
TICKS_PER_M = TICKS_PER_MM * 1000.0 # encoder ticks per meter


WHEEL_BASE_MM = 50.0 # mm
WHEEL_BASE_M = WHEEL_BASE_MM / 1000.0 # m

MEASUREMENT_PERIOD_S = 0.05 # s



