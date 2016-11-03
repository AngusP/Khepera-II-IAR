#!/usr/bin/env python

#
#     - - - -  I A R  - - - -
#
#  s1311631         Angus Pearson
#  s1346981    Jevgenij Zubovskij
#

from navigation_state import Navigation_State
from AStar.py import AStar
from AStar.py import Cell

import constants
import sys

import time
import math





class Bug_Algorithm:


	def __init__(self):
		self.aStar = AStar()
	
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
	def get_angle_on_m(self, odo_state, direction):
		direction_angle = math.atan2(direction[1] , direction[0])
		#if no difference, well then we never left the spot 
		vector_magnitude = self.vector_magnitude(direction)
		if vector_magnitude == 0:
			return 0
		
		direction_angle = math.degrees(direction_angle) - self.normalize_angle(math.degrees(odo_state.theta))
		return self.normalize_angle(direction_angle)
	
  
	
    #TODO placeholder method, before any linearization etc.
    def is_away_from_path(self, direction):
			return vector_magnitude(self, direction) > constants.AWAY_FROM_PATH
		
        
    
	#return new state
    def new_state(self, nav_state, odo_state, grid_state, grid):
	     
		current_cell 	 = grid_state.current_cell
		next_cell    	 = grid_state.active_path[0]
		direction = (next_cell.x - current_cell.x, next_cell.y - current_cell.y)
	    
        speed_l = nav_state.speed_l 
	    speed_r = nav_state.speed_r
		
		#turn aggressively
		turn_less = -constants.CONST_SPEED 
		turn_more = constants.CONST_SPEED 

		#TODO make it say got to another nest or something if there is another one

		#recalculate path if for some reason strayed from it too far
		if self.is_away_from_path(direction):
			grid_state.active_path = aStar.replan(current_cell, (0,0) , grid) 


		angle_to_m = self.get_angle_on_m(odo_state, direction)
		#OUR angle too small
		if angle_to_m > constants.M_N_ANGLE:		
				
				#no need to check for obstacles as pathing takes cares of it for us
				speed_r = turn_more
				speed_l = turn_less	

		#OUR angle too big
		elif angle_to_m < -constants.M_N_ANGLE:
				#no need to check for obstacles as pathing takes cares of it for us
				speed_r = turn_less
				speed_l = turn_more

		#send out the new speed controls			
		nav_state.speed_l = speed_l
		nav_state.speed_r = speed_r 
		    
