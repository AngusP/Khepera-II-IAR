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
	
  
    #TODO check
    def is_away_from_path(self, direction):
		#if we are too far away in cells
		return self.vector_magnitude(direction) > constants.AWAY_FROM_PATH

	
    def cell_transition(self, pathing_state, odo_state):

			
		#next_cell_pose = pathing_state.active_path[1].get_pose()
		current_pose = (odo_state.x), self.snap(odo_state.y)
		current_cell = pathing_state.get_cell(current_pose)
		
		#TODO if cell = None, throw an error
		#TODO NOTE that the path INCLUDES actual end and start states

		if pathing_state.active_path[1] == current_cell:
			#pop the element at index 0
			pathing_state.active_path.pop(0)
			#update current cell
		
		#update current cell
		pathing_state.current_cell = current_cell


	
	
    #method returning out current heading cell	
    def get_direction(self, odo_state, pathing_state):
		heading_to    	 = pathing_state.active_path[1].get_pose()
		direction 	 = (heading_to[0] - odo_state.x, heading_to[1] - odo_state.y)
		return direction


    #method to record the grid location of a new food_source
    def drive_over_food(self, pathing_state, comms):
		#now the pathing is definitely active
		pathing_state.algorithm_activated = True
		#add current cell as a food source
		pathing_state.add_food_source()
		#indicate picking up food
		comms.blinkyblink()
		#give it time to actually pick up the food
		time.sleep(0.5)
		#set food source as picked
		pathing_state.pick_food_up()
	
		
    #when occupancies change, we replan our route 
    def update_pathing_grid(self, pathing_state, changed_occupancies):
	pathing_state.update_grid(changed_occupancies)
	self.replan_sequence(pathing_state)
		
    #planning after a piece of food is collected    
    def replan_sequence(self, pathing_state):

	start = pathing_state.currrent_cell.get_coordinates()
	end = (0,0)
	
	#check if have more food sources to collect
	return_home = not pathing_state.has_uncollected_food()
	if not return_home:
		food = pathing_state.get_closest_food()
		end  = food.cell.get_coordinates()

	x_neg = pathing_state.planning_grid.x_neg
	y_neg = pathing_state.planning_grid.y_neg

	start = (start[0]+x_neg, start[1]+y_neg)
	end = 	(end[0]	 +x_neg, end[1]	 +y_neg)

	
	#get new path
 	pathing_state.active_path = self.aStar.replan(start, end , pathing_state.planning_grid)
	
		
    #return new state
    def new_state(self, nav_state, odo_state, pathing_state):
	     
		#get current grid cell location
		self.cell_transition(pathing_state, odo_state)

		#if at the nest
		if pathing_state.current_cell.get_coordinates() == (0,0):
			#indicate completion
			pathing_state.done = True

			#TODO replan here when algorithm works fully
			return
			
			
		direction = self.get_direction(odo_state, pathing_state)
	    
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
	        direction  = self.get_direction(odo_state, pathing_state)
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
		    
