# ODOMETRY V1 (no calibration, dumb formulas based on encoders)
# USES https://www.cs.princeton.edu/courses/archive/fall11/cos495/COS495-Lecture5-Odometry.pdf

from odometry_state import Odometry_State
import constants
from state import GenericState

import math

import sys
import time

class Odometry_Algorithm_1(GenericState):
        def __init__(self):
                GenericState.__init__(self)

	def delta_s(self, delta_odo):
		result 		= 0
		result 		= ((delta_odo[0] + delta_odo[1]) / 2 ) / constants.TICKS_PER_M 
	
		return result # m

	def delta_theta(self, delta_odo):
		result 		= 0
		result 		= ((delta_odo[1] - delta_odo[0]) / constants.WHEEL_BASE_M ) / constants.TICKS_PER_M 
	
		return result #radians

	def delta_x_y_angle(self, curr_theta, delta_odo):
		result 	    = [0]*3
	
		delta_dist  = self.delta_s(delta_odo)
		delta_angle = self.delta_theta(delta_odo)
		delta_x     = delta_dist*math.cos(curr_theta + delta_angle / 2)
	    	delta_y     = delta_dist*math.sin(curr_theta + delta_angle / 2)
	    
	    	#altetrnative (still an approximation) is 
	    	#delta_x = delta_dist*math.cos(curr_theta + delta_angle )
	    	#delta_y = delta_dist*math.sin(curr_theta + delta_angle )
	    
	
		result[0] = delta_x
		result[1] = delta_y
		result[2] = delta_angle
	
		return result

	def new_state(self, prev_state, delta_odo):

		state_change = self.delta_x_y_angle(prev_state.theta, delta_odo)
	    
		prev_x = prev_state.x
		prev_y = prev_state.y
		prev_theta = prev_state.theta
		t = constants.MEASUREMENT_PERIOD_S
		
		x_n 	= prev_x     + state_change[0]
		y_n 	= prev_y     + state_change[1]
		theta_n	= prev_theta + state_change[2]


	
		#return new state
		result 	    = Odometry_State() 
		result.time = prev_state.time + t
		result.x    = x_n
		result.y    = y_n
	
		result.theta 	 = ((theta_n % (2*math.pi)) / (2*math.pi)) * 360

		return result


	
	






