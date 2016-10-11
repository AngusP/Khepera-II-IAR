# Class to define state to not have arrays... cause arrays are ugly
class Odometry_State:
  # time when state recorded
  time 	= 0
  # x location relative to initial placament at time of recording
  x 	= 0
  # y location relative to initial placament at time of recording
  y 	= 0
  # y location relative to initial placament at time of recording
  theta = 0
  #odometry values
  odo = [0]*2
  #ir values
  nav = [0]*8

