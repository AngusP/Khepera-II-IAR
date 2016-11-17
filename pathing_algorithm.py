#!/usr/bin/env python

#
#     - - - -  I A R  - - - -
#
#  s1311631         Angus Pearson
#  s1346981    Jevgenij Zubovskij
#


from astar import AStar
from astar import Cell
from pathing_state import Food_Source

import constants
import sys
import math
import time

class Pathing_Algorithm:


    def __init__(self, planning_granularity, data_getter):

		self.aStar = AStar(planning_granularity, data_getter)
		self.granularity = planning_granularity
	
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
	
    #determines if have left the path
    def is_away_from_path(self, direction):
		#if we are too far away by several cells from the cell it should be heading to
		return self.vector_magnitude(direction) > self.granularity*2
   
    #snap passed actual X or Y value to a math planning grid granularity
    def snap(self, value):
        return ( int(value) / self.granularity) * self.granularity
	
    #method determining if we have moved to a new cell
    def cell_transition(self, pathing_state, odo_state):
			
		#next_cell_pose = pathing_state.active_path[1].get_pose()
		current_coordinates = (self.snap(odo_state.x), self.snap(odo_state.y))

		#we know the cell is unboccupied as we are on it
		pathing_state.current_cell = Cell(current_coordinates[0], current_coordinates[1] , True)	

		#check if we have anywhere to go in the firts place
		if len(pathing_state.active_path) == 1 or not pathing_state.algorithm_activated:
			return

		#check if we have advanced along the path
		if pathing_state.active_path[1].get_coordinates() == current_coordinates:
			#pop the element at index 0
			pathing_state.active_path.pop(0)
			#update current cell
			pathing_state.current_cell = pathing_state.active_path[0]
	
	
    #method returning out current heading cell	
    def get_direction(self, odo_state, pathing_state):
		heading_to    	 = pathing_state.active_path[1].get_coordinates()
		direction 	 = (heading_to[0] - odo_state.x, heading_to[1] - odo_state.y)
		return direction


    #method to record a new food source and collect it, or just collect current one
    def drive_over_food(self, pathing_state, comms):

	if pathing_state.are_on_food() != None:
		#stop first
		comms.drive(0,0)
		#wait to simulate picking up food
		time.sleep(constants.WAIT_PERIOD_S)
		comms.drive(constants.CONST_SPEED,constants.CONST_SPEED)
		#set food source as picked
		pathing_state.pick_food_up()
		#replan, maybe a more efficient route now available
		self.replan_sequence(pathing_state)


    def add_food_source(self, pathing_state, comms):

	#now the pathing is definitely active
	pathing_state.algorithm_activated = True
	#add current cell as a food source
	pathing_state.add_food_source()
	#now collect it
	self.drive_over_food(self, pathing_state, comms)

		
    #planning after a piece of food is collected    
    def replan_sequence(self, pathing_state):

	start = pathing_state.current_cell.get_coordinates()
	end = (0,0)
	
	#check if have more food sources to collect
	return_home = not pathing_state.has_uncollected_food()

	#if do have such food sources, then collect them before going home
	if not return_home:
		food = pathing_state.get_closest_food()
		end  = food.location.get_coordinates()

	#get new path
 	pathing_state.active_path = self.aStar.replan(start, end)

	print "REPLANNING {} to {}".format(start, end)
	
		
    #return new state
    def new_state(self, nav_state, odo_state, pathing_state, comms):
	     
		#alwayts update location on grid
		self.cell_transition(pathing_state, odo_state)

		#if algorithm not active or we do not have control, do not waste computational power
		if not (pathing_state.algorithm_activated and nav_state.yielding_control):
			return nav_state

		#if at the nest and have no more places to go
		if pathing_state.current_cell.get_coordinates() == (0,0) and len(pathing_state.active_path) == 1:

			#indicate completion of round
			print("Brought %d food back to nest" % pathing_state.food)
			comms.drive(0, 0)

			#drop off food, reset food nodes
			pathing_state.drop_off_food()
			#indicate visually
			comms.blinkyblink()
			comms.drive(0,0)
			#wait to simulate actual dropping of food
			time.sleep(constants.WAIT_PERIOD_S)
			#drive forward 
			comms.drive(constants.CONST_SPEED,constants.CONST_SPEED)
			return nav_state


		#try to begin collecting food
		self.drive_over_food(pathing_state, comms)

		# do not interrupt the spiral around found food source
		if pathing_state.spiral:	
			pathing_state.spiral_counter += 1
			#make it got in a spiral
			nav_state.speed_l = pathing_state.spiral_counter / constants.SPIRAL_SPEED_COEFFICIENT
			nav_state.speed_r = constants.CONST_SPEED 
			return nav_state
	
		
		#get current direction vector to heading grid cell	
		direction = self.get_direction(odo_state, pathing_state)
	    
                speed_l = nav_state.speed_l 
	        speed_r = nav_state.speed_r
		
		#turn aggressively
		turn_less = -constants.CONST_SPEED 
		turn_more = constants.CONST_SPEED 


		#recalculate path if for some reason strayed from it too far
		if self.is_away_from_path(direction):

			#print "FAR AWAY REPLAN"
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
		    
