#
#     - - - -  I A R  - - - -
#
#  s1311631         Angus Pearson
#  s1346981    Jevgenij Zubovskij
#

import math

def ir_to_dist(reading):
    '''
    Takes a Khepera IR sensor reading, reeturns the distance that represents.

    From solved equation:
    y = 1.074519 + (10.57748 - 1.074519)/(1 + ( x /70.42612)^69.9039)^0.02119919
    '''
    return 10.0 * ( 1.074519 + (10.57748 - 1.074519)
                    /
                    math.pow(1 + (math.pow((reading / 70.42612),69.9039)), 0.02119919 ))


