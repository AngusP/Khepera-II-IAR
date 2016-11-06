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



def relative_to_fixed_frame_tf(xabs, yabs, thetaabs, x, y):
    '''
    Transform points x, y relative to xabs and yabs with rotation
    thetaabs relative to the fixed frame.

    e.g. the Khepera's pose is (xabs, yabs, theta), and a distance sensor
    reading is at (x,y) relative to the Khepera's axes

    Doctests:

    >>> relative_to_fixed_frame_tf(-10, -10, 0, 10, 10)
    (0.0, 0.0)

    >>> relative_to_fixed_frame_tf(10, 10, 0, -10, -10)
    (0.0, 0.0)
    
    >>> relative_to_fixed_frame_tf(100, 0, math.pi, 100, 0)
    (0.0, -0.0)

    >>> relative_to_fixed_frame_tf(0, 100, math.pi, 0, 100)
    (-0.0, 0.0)

    >>> relative_to_fixed_frame_tf(20, 10, math.pi/2, 5, 2.5)
    (12.5, 15.0)

    >>> relative_to_fixed_frame_tf(-20, -15, 3 * math.pi/2, 10, 5)
    (-25.0, -5.0)
    '''
    return tuple(map(lambda x: round(x, 4),
                     _relative_to_fixed_frame_tf_UNROUND(xabs, yabs, thetaabs, x, y)))


def _relative_to_fixed_frame_tf_UNROUND(xabs, yabs, thetaabs, x, y):
    '''
    Unsafe version of non underscored method relative_to_fixed_frame_tf(),
    won't round return variables. May be useful, may cause sh*t to explode


    2D Transformation
    |  Cos(a), Sin(a) | . |x|
    | -Sin(a), Cos(a) |   |y|
    '''
    
    tfx = (x *  math.cos(thetaabs)) + (y * math.sin(thetaabs))
    tfy = (x * -math.sin(thetaabs)) + (y * math.cos(thetaabs))
    
    tfx += xabs
    tfy += yabs

    return tfx, tfy


if __name__ == '__main__':
    # Run documentation tests
    import doctest
    doctest.testmod()
