#!/usr/bin/env python

#
#     - - - -  I A R  - - - -
#
#  s1311631         Angus Pearson
#  s1346981    Jevgenij Zubovskij
#

from navigation_state import Navigation_State

import constants
import sys

import time
import math

class Bug_Algorithm:


	def lineMagnitude (self, x1, y1, x2, y2):
	    lineMagnitude = math.sqrt(math.pow((x2 - x1), 2)+ math.pow((y2 - y1), 2))
	    return lineMagnitude
		
	#vector magnitude calculator
	def vector_magnitude(self, vector):
		return math.sqrt( math.pow(vector[0],2) + math.pow(vector[1],2))
		

	#normalizes angle in degrees to -180 : 180 degrees
        def normalize_angle(self, angle):
		if angle < 0:
		    angle = angle % -360
		    if angle < -180:
			angle = 360 + angle
		else:
		    angle = angle % 360
		    if angle > 180:
			angle = -(360 - angle)
		return angle

	#vector difference calculator
	def vector_diff(self, vector_1, vector_2):
		dx = vector_1[0] - vector_2[0]*1.0
		dy = vector_1[1] - vector_2[1]*1.0
		return [dx,dy]

		 
	#get angle while ON the M-line
	def get_angle_on_m(self, odo_state, bug_state):
		direction = self.vector_diff( bug_state.m_line_start,  bug_state.m_line_end)
		direction_angle = math.atan2(direction[1] , direction[0])
		#if no difference, well then we never left the spot 
		vector_magnitude = self.vector_magnitude(direction)
		if vector_magnitude == 0:
			return 0
		
		direction_angle = math.degrees(direction_angle) - self.normalize_angle(math.degrees(odo_state.theta))
		return self.normalize_angle(direction_angle)
	
    
    #TODO placeholder method, before any linearization etc.
    def is_away_from_path(self, grid_state, grid):
    def is_left_free(self, grid_state, grid):
    def is_right_free(self, grid_state, grid):
        
    
	#return new state
    def new_state(self, grid_state, grid):
	                
	    
        speed_l = nav_state.speed_l 
	    speed_r = nav_state.speed_r

		#do not turn if not needed

		turn_less = constants.CONST_SPEED 
		turn_more = constants.CONST_SPEED

		#recalculate M-line if such a need arises
		if self.is_away_from_path(nav_state):
			
			
			#turn aggressively
			turn_less = -constants.CONST_SPEED 
			turn_more = constants.CONST_SPEED 

			angle_to_m = self.get_angle_on_m(odo_state, bug_state)

			#OUR angle too small
			if angle_to_m > constants.M_N_ANGLE:		
				
					# check if there is a wall on the left
					if self.is_left_free(grid_state, grid):
						# if there is none, turn left
						speed_r = turn_more
						speed_l = turn_less	

			#OUR angle too big
			elif angle_to_m < -constants.M_N_ANGLE:

					# check if there is a wall on the right
					if self.is_right_free(grid_state, grid):
						# if there is none, turn right
						speed_r = turn_less
						speed_l = turn_more

		#send out the new speed controls			
		nav_state.speed_l = speed_l
		nav_state.speed_r = speed_r 
		return nav_state
		    
