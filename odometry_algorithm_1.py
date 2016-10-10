# ODOMETRY V1 (no calibration, dumb formulas based on encoders)
# USES https://www.cs.princeton.edu/courses/archive/fall11/cos495/COS495-Lecture5-Odometry.pdf

from comms import Comms
from odometry_state import Odometry_State
import constants

import math

import sys
import time

class Odometry_Algorithm_1:
	def delta_l_r(self, prev_l, prev_r):
		result 	  = [0]*2
		odo 	  = comms.get_odo()
		delta_l   = prev_l - odo[0]
		delta_r   = prev_r - odo[1]
	
		result[0] = delta_l
		result[1] = delta_r
		return result

	def delta_s(self, prev_l, prev_r):
		result 		= 0
	    	delta_odo 	= delta_l_r(prev_l, prev_r)
		result 		= ((delta_odo[0] + delta_odo[1]) / 2 ) / constants.TICKS_PER_M 
	
		return result # m

	def delfa_theta(self, prev_l, prev_r):
		result 		= 0
	    	delta_odo 	= delta_lr(prev_l, prev_r)
		result 		= ((delta_odo[1] - delta_odo[0]) / constants.WHEEL_BASE_M ) / constants.TICKS_PER_M 
	
		return result #radians

	def delta_x_y_angle(self, prev_l, prev_r, curr_theta):
		result 	    = [0]*3
	
		delta_dist  = delta_s(prev_l, prev_r)
		delta_angle = delta_theta(prev_l, prev_r)
		delta_x     = delta_dist*math.cos(curr_theta + delta_angle / 2)
	    	delta_y     = delta_dist*math.sin(curr_theta + delta_angle / 2)
	    
	    	#altetrnative (still an approximation) is 
	    	#delta_x = delta_dist*math.cos(curr_theta + delta_angle )
	    	#delta_y = delta_dist*math.sin(curr_theta + delta_angle )
	    
	
		result[0] = delta_x
		result[1] = delta_y
		result[2] = delta_angle
	
		return result

	def new_state(self, prev_state):

		state_change = delta_x_y_angle(prev_state.encoder_l, prev_state.encoder_r, prev_state.theta)
	    
		x_n 	= prev_x     + state_change[0]
		y_n 	= prev_y     + state_change[1]
		theta_n	= prev_theta + state_change[2]
	
	    	odo 	= comms.get_odo()
	
		#return new state
		result 	    = Odometry_State() 
		result.time = prev_state.time + t
		result.x    = x_n
		result.y    = y_n
	
		result.theta 	 = theta_n
		result.encoder_l = odo[0]
		result.encorer_r = odo[1]

		return result


	
	






