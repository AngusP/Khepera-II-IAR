#!/usr/bin/env python

#
#     - - - -  I A R  - - - -
#
#  s1311631         Angus Pearson
#  s1346981    Jevgenij Zubovskij
#


from AStar import AStar
from AStar import Cell
from pathing_state import Food_Source

import constants
import sys
import math
import time

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
		
		# can do trigonometric distance estimation as know cells equally spaced form each other
		# and know the coordinates of one of their corners
	
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
		return self.vector_magnitude(direction) > constants.AWAY_FROM_PATH



	
    def check_cell_transition(self, pathing_state, odo_state):


		#if we are in the last cell, we have completed the path
		if len(pathing_state.active_path) == 1:
			#indicate completion
			pathing_state.done = True
			return
		

		current_cell 	 = pathing_state.current_cell
		next_cell    	 = pathing_state.active_path[1]
		
		dist_to_next	 = (next_cell.x - odo_state.x, next_cell.y - odo_state.y)
			
		grid_detected 	  = current_cell == next_cell
		odometry_detected = self.vector_magnitude(dist_to_next) < constants.IN_CELL



		#so if reached the next cell, well pop the now-previous-one
		if grid_detected or odometry_detected:
			#pop the element at index 0
			pathing_state.active_path.pop(0)


	
	
    #method returning out current heading cell	
    def get_direction(self, pathing_state):
		current_cell 	 = pathing_state.current_cell
		head_cell    	 = pathing_state.active_path[0]
		direction 	 = (head_cell.x - current_cell.x, head_cell.y - current_cell.y)
		return direction


    #method to record the grid location of a new food_source
    def drive_over_food(self, pathing_state, comms):
		#now the pathing is definitely active
		pathing_state.algorithm_activated = True

		#add current cell as a food source
		food_source = pathing_state.current_cell
		#if found a new food source
		if food_source not in pathing_state.food_sources:
			pathing_state.food_sources.append(Food_Source(food_source))


		#indicate picking up food
		comms.blinkyblink()
		#give it time to actually pick up the food
		time.sleep(0.5)
		#records food picked up
		pathing_state.food += 1
		#set food source as picked
		self.set_food_source_picked(pathing_state, food_source)

    #TODO check if works by reference		
    def set_food_source_picked(self, pathing_state, grid_cell):

		for food_source in pathing_state.food_sources:
			if food_source.location == grid_cell:		
				food_source.picked_up = True
		
		



  

    #TODO
    def update_pathing_grid(self):
	print('a')
		
    
    #TODO
    def replan_sequence(self, pathing_state):
	#TODO make replanning to say a different goal
	#TODO make it say got to another nest or something if there is another one
	#TODO check if are in any other node along the path... or fucking not....
	
	#TODO uncomment for actual operation AND add to have no computation if already in destination cell
	#pathing_state.active_path = self.aStar.replan(pathing_state.current_cell, Cell(0,0,True) , pathing_state.grid) 	
	
 	pathing_state.active_path = self.aStar.replan((0,0), (5,5) , pathing_state.grid)
	print(pathing_state.active_path)
	#TODO remove after debug
	print("REPLANNED")
	
		
    #return new state
    def new_state(self, nav_state, odo_state, pathing_state):
	     
		#TODO check maybe somehow if we are within a cell or something
		
		self.check_cell_transition(pathing_state, odo_state)

		#TODO replan here or something
		if pathing_state.done == True:
			return
			
		direction = self.get_direction(pathing_state)
	    
                speed_l = nav_state.speed_l 
	        speed_r = nav_state.speed_r
		
		#turn aggressively
		turn_less = -constants.CONST_SPEED 
		turn_more = constants.CONST_SPEED 


		#recalculate path if for some reason strayed from it too far
		if self.is_away_from_path(direction):
			#so get the new path
			self.replan_sequence(pathing_state)
			

		#renew our direction
	        direction  = self.get_direction(pathing_state)
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
		return nav_state
		    
