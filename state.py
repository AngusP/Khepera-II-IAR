# ODOMETRY V1 (no calibration, dumb formulas based on encoders)
# USES https://www.cs.princeton.edu/courses/archive/fall11/cos495/COS495-Lecture5-Odometry.pdf

from math import *

TICKS_PER_M = 12000 # encoder ticks per meter
WHEEL_BASE_M = 0.01 # m
MEASUREMENT_PERIOD_S = 0.02 # s


# Class to define state to not have arrays... cause arrays are ugly
class GenericState(object):
  
  def __init__(self):
    self.time = 0
    # x location relative to initial placament at time of recording
    self.x = None
    # y location relative to initial placament at time of recording
    self.y = None
    # y location relative to initial placament at time of recording
    self.theta = None
    # encoder values at the time of recording
    self.encoder_l = None
    self.encoder_r = None

  def delta_l_r(self, prev_l, prev_r):
    raise NotImplementedError

  def delta_s(self, prev_l, prev_r):
    raise NotImplementedError

  def delfa_theta(self, prev_l, prev_r):
    raise NotImplementedError

  def delta_x_y_angle(self, prev_l, prev_r, curr_theta):
    raise NotImplementedError

  def new_state(self, prev_state):
    raise NotImplementedError

# Subclass
class OtherState(GenericState):
  def __init__(self):
    GenericState.__init__(self)
