
#
#     - - - -  I A R  - - - -
#
#  s1311631         Angus Pearson
#  s1346981    Jevgenij Zubovskij
#

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
