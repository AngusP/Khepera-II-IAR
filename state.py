
#
#     - - - -  I A R  - - - -
#
#  s1311631         Angus Pearson
#  s1346981    Jevgenij Zubovskij
#

# Class to define state to not have arrays... cause arrays are ugly
class GenericState(object):
  
  def __init__(self):
    self.time = None
    # x location relative to initial placament at time of recording
    self.x = None
    # y location relative to initial placament at time of recording
    self.y = None
    # y location relative to initial placament at time of recording
    self.theta = None

# Subclass
class OtherState(GenericState):
  def __init__(self):
    GenericState.__init__(self)
