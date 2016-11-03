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
import math

class Pathing_Algorithm:


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
		
		#CAN DO TRIG AS WE KNOW CELL COORDINATES
	
		direction_angle = math.atan2(direction[1] , direction[0])
		#if no difference, well then we never left the spot 
		vector_magnitude = self.vector_magnitude(direction)
		if vector_magnitude == 0:
			return 0
		
		direction_angle = math.degrees(direction_angle) - self.normalize_angle(math.degrees(odo_state.theta))
		return self.normalize_angle(direction_angle)
	
  
	
    #TODO placeholder method, before any linearization etc.
    def is_away_from_path(self, direction):
			#if we are too far away in cells
			return vector_magnitude(self, direction) > constants.AWAY_FROM_PATH
	
	def is_in_next_cell(self, grid_state, odo_state)
			current_cell 	 = grid_state.current_cell
		    next_cell    	 = grid_state.active_path[1]
		
			dist_to_next	 = (next_cell.x - odo_state.x, next_cell.y - odo_state.y)
			
			grid_detected = current_cell == next_cell
			odometry_detected = self.vector_magnitude(self, dist_to_next) < constants.IN_CELL
			
			return grid_detected or odometry_detected
	
	
	#method returning out current heading cell	
    def get_direction(self, grid_state):
			current_cell 	 = grid_state.current_cell
		    head_cell    	 = grid_state.active_path[0]
		    direction 		 = (head_cell.x - current_cell.x, head_cell.y - current_cell.y)
    
	#return new state
    def new_state(self, nav_state, odo_state, grid_state, grid):
	     
		#TODO check maybe somehow if we are within a cell or something
		#TODO rename grid_state in pathing_state...maybe
		#TODO plan sequence of paths
		
		#so if reached the next cell, well pop the now-previous-one
		if self.is_in_next_cell(grid_state, odo_state):
			
			#if we are in the last cell, we have completed the path
			if len(grid_state.active_path) < 1
				#indicate completion
				grid_state.done = True
				return
			#proceed to check ehading
			grid_state.active_path.pop()
		
		direction        = self.get_direction(grid_state)
	    
        speed_l = nav_state.speed_l 
	    speed_r = nav_state.speed_r
		
		#turn aggressively
		turn_less = -constants.CONST_SPEED 
		turn_more = constants.CONST_SPEED 

		#TODO make it say got to another nest or something if there is another one
		#TODO check if are in any other node along the path... or fucking not....
		
		#recalculate path if for some reason strayed from it too far
		if self.is_away_from_path(direction):
			#so get the new path
			grid_state.active_path = self.aStar.replan(current_cell, (0,0) , grid) 

		#renew our direction
	    direction  = self.get_direction(grid_state)
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
		    
