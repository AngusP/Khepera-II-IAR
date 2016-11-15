#
#     - - - -  I A R  - - - -
#
#  s1311631         Angus Pearson
#  s1346981    Jevgenij Zubovskij
#

from astar import Cell
import math


class Food_Source:
   #make a new structure to indicate if the food was picked up in this round
   def __init__(self, location_cell):
	self.location = location_cell
	self.picked_up = False


   #custom print
   def __str__(self):
	return self.location.__str__() + " " + str(self.picked_up)
 
   #custom comparator
   def __eq__(self, other):
        #reachability should not change during planning, so only X,Y matter for comparison
	if other is None:
		return False
        return self.location == other.location

   #custom comparator
   def __ne__(self, other):
        return not self.__eq__(other)

class Pathing_State:

    def __init__(self):
           #variable for current path
           self.active_path = []
           
           #variable for current path
           self.current_cell = Cell(0,0, True)
           
           #variable for say alternative food sources
           self.food_sources = []
           
           #variable to indicate reached current path goal
           self.done = False
           
           #variable to indicate it is active
           self.algorithm_activated = False

	   #idnicating number of food items carried
	   self.food = 0	

   
	   #indicate have not explored randomly yet this food collecting round
	   self.explored = False


	   #indicate the spirallign around point of interest (food source)
	   self.spiral = True
	   self.spiral_counter = 0
	   self.spiral_source = None

	   
    #return if the exploration was done for this round
    def exploration_not_done(self):
		return not self.explored
	
    #denote we have already explored this round	
    def complete_exploration(self):
	        self.explored = True

    #function indicatign spiralling start around food source until indicated food was collected
    def start_spiral(self, food_source):
		#set variables
		self.spiral = True
		self.spiral_counter = 0
		self.spiral_source = food_source

    #indicate spiral end
    def end_spiral(self)
		#reset varaibles
		self.spiral = False
		self.spiral_counter = 0
		self.spiral_source = None

    #add a food source on current grid space if such does not exist yet
    def add_food_source(self):
	   food_source = Food_Source(self.current_cell)
	   if food_source not in self.food_sources:
	   	self.food_sources.append(food_source)
	   

    #check if has uncollected food sources
    def has_uncollected_food(self):
	   for food_source in self.food_sources:
		if not food_source.picked_up:
			return True
	   return False

    #distance between cells
    def cell_distance(self, from_cell, to_cell):
	   #euclidian distance
	   x_diff = from_cell.x - to_cell.x
	   y_diff = from_cell.y - to_cell.y
	   return math.sqrt( math.pow(x_diff,2) + math.pow(y_diff,2))
	  

    #get closest uncollected food
    def get_closest_food(self):
	   
	   closest_food = None
	   closest_dist = 0

	   for food_source in self.food_sources:
		if not food_source.picked_up:
			#get the closest one
			distance = self.cell_distance(self.current_cell, food_source.location)
			if closest_food is None or distance <= closest_dist:
				closest_food = food_source

	   return closest_food
	
    #pick up the food on the spot the command was called if there is an unpicked one
    def pick_food_up(self):
	for food_source in self.food_sources:

		#if have food sources in current cell
		if food_source.location == self.current_cell and not food_source.picked_up:		
			#collect food and record that
			food_source.picked_up = True
			self.food += 1

    #specific method for spiral food pick up
    def pick_up_spiral_food(self):
	for food_source in self.food_sources:
		#if uncollceted the spiral source food
		if food_source.location == self.spiral_source.location and not food_source.picked_up:		
			#collect food and record that
			food_source.picked_up = True
			self.food += 1
	
	

    #drop off food at the nest and finish
    def	drop_off_food(self):

	#reset exploration for next round
	self.explored = False
	#indicate we are at the nest
	self.food = 0
	#drop off food
	self.done = True
	#reset food sources
	self.algorithm_activated = False
	#self.print_food()
	for food_source in self.food_sources:
		food_source.picked_up = False

    #return if we in the same cell as a uncollected food source
    def are_on_food(self):
	for food_source in self.food_sources:
		if food_source.location == self.current_cell and not food_source.picked_up:
			return food_source

	return None		
    
