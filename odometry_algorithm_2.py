#ODOMETRY V2, USES https://www.cs.cmu.edu/afs/cs.cmu.edu/academic/class/16311/www/s07/labs/NXTLabs/Lab%203.html


#TODO check previous methods for SI system
#TODO check if the thing works for equal speeds on both wheels (and previous solutions as well)

from odometry_state import Odometry_State


from state import GenericState
import constants

import math

import sys
import time

class Odometry_Algorithm_2(GenericState):
        def __init__(self):
                GenericState.__init__(self)
	
	def velocity_l_r(self, delta_odo):
	
		result = [0]*2
	
		speed_l = (delta_odo[0] / constants.TICKS_PER_M) /  constants.MEASUREMENT_PERIOD_S # m / s
		speed_r = (delta_odo[1] / constants.TICKS_PER_M) /  constants.MEASUREMENT_PERIOD_S # m / s
	
		result[0] = speed_l
		result[1] = speed_r
	
		return result
	
	def velocity_linear_angular(self, delta_odo):
		result = [0]*2
	
		velocity_left_right = self.velocity_l_r(delta_odo)
	
		velocity_linear = (velocity_left_right[0] + velocity_left_right[1] ) / float(2) # m /s
		velocity_angular = (velocity_left_right[1] - velocity_left_right[0] ) / constants.WHEEL_BASE_M # rad / s
	
		result[0] = velocity_linear
		result[1] = velocity_angular
	
		return result
	
	
	def new_state(self, prev_state, delta_odo):
	
		velocities = self.velocity_linear_angular(delta_odo)
	
		v = velocities[0]
		w = velocities[1]
		t = constants.MEASUREMENT_PERIOD_S
	
		prev_theta = prev_state.theta
		prev_x = prev_state.x
		prev_y = prev_state.y

	
		#apply the equation
		k00 = v * math.cos(prev_theta)
		k01 = v * math.sin(prev_theta)
		k02 = w
	
		k10 = v * math.cos(prev_theta + t * k02/float(2))
		k11 = v * math.sin(prev_theta + t * k02/float(2))
		k12 = w
	
		k20 = v * math.cos(prev_theta + t * k12/float(2))
		k21 = v * math.sin(prev_theta + t * k12/float(2))
		k22 = w	

		k30 = v * math.cos(prev_theta + t * k22/float(2))
		k31 = v * math.sin(prev_theta + t * k22/float(2))
		k32 = w		
	
		x_n 	= prev_x     + t/6 * (k00 + 2*(k10 + k20) + k30)
		y_n 	= prev_y     + t/6 * (k01 + 2*(k11 + k21) + k31)
		theta_n = prev_theta + t/6 * (k02 + 2*(k12 + k22) + k32)

		#return new state
		result = Odometry_State() 
		result.time = prev_state.time + t
		result.x = x_n
		result.y = y_n

		#print(theta_n)
		result.theta = theta_n 

	
		return result

	
	
	
	
	
	
	






