#!/usr/bin/env python

#
#     - - - -  I A R  - - - -
#
#  s1311631         Angus Pearson
#  s1346981    Jevgenij Zubovskij
#


from odometry_state import Odometry_State
import constants


import math

import sys
import time

class Odometry_Algorithm:
        def __init__(self):
                pass

	#calculate differences in distance driven by different wheels
	def delta_s(self, delta_odo):

		result 		= 0
		result 		= ((delta_odo[0] + delta_odo[1]) / float(2) ) / constants.TICKS_PER_MM

		return result # mm

	#calculate the orientation angle
	def delta_theta(self, delta_odo):

		result 		= 0
		result 		= ((delta_odo[1] - delta_odo[0]) / constants.TICKS_PER_MM) / constants.WHEEL_BASE_MM 
	

		return result #radians

	#calculate the change in angle, X and Y
	def delta_x_y_angle(self, curr_theta, delta_odo):
		result 	    = [0]*3
	
		delta_dist  = self.delta_s(delta_odo)
		delta_angle = self.delta_theta(delta_odo)

		
		new_angle = curr_theta + delta_angle / float(2) # this is the alternative
		

		delta_x     = delta_dist*math.cos(new_angle) # in mm
	    	delta_y     = delta_dist*math.sin(new_angle) # in mm
	    
		
		result[0] = delta_x
		result[1] = delta_y
		result[2] = delta_angle
		
	
		return result


	#get the new state
	def new_state(self, prev_state, new_odo):
		
		delta_odo = [new_odo[0] - prev_state.odo[0], new_odo[1] - prev_state.odo[1] ]
		state_change = self.delta_x_y_angle(prev_state.theta, delta_odo)
	    
 		#update the variables
		prev_x = prev_state.x
		prev_y = prev_state.y
		prev_theta = prev_state.theta
		t = constants.MEASUREMENT_PERIOD_S
		
		#calculate the new values
		x_n 	= prev_x     + state_change[0]
		y_n 	= prev_y     + state_change[1]
		theta_n	= prev_theta + state_change[2]


	
		#return new state
		result 	     = Odometry_State() 
		result.time  = prev_state.time + t
		result.x     = x_n
		result.y     = y_n
		result.odo   = new_odo
		result.theta = theta_n

		return result


	
	






