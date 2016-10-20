#
#     - - - -  I A R  - - - -
#
#  s1311631         Angus Pearson
#  s1346981    Jevgenij Zubovskij
#


from state import GenericState

class Odometry_State(GenericState):
  def __init__(self):
    GenericState.__init__(self)
    # time when state recorded
    self.time 	= 0
    # x location relative to initial placement at time of recording
    self.x 	= 0
    # y location relative to initial placement at time of recording
    self.y 	= 0
    # y location relative to initial placement at time of recording
    self.theta  = 0

    self.odo = [0]*2
    #ir values
    self.nav = [0]*8

