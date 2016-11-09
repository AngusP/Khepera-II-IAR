#
#     - - - -  I A R  - - - -
#
#  s1311631         Angus Pearson
#  s1346981    Jevgenij Zubovskij
#

from AStar import Cell
import numpy
import math


class Food_Source:
   #make a new structure to indicate if the food was picked up in this round
   def __init__(self, location_cell):
	self.location = location_cell
	self.picked_up = False

class Planning_Grid:
   def __init__(self):

	#Only know where we ourselves are and that cell is unoccupied
	self.grid = numpy.zeros((1,1), dtype=int)
	
	#the size of the matrix
	self.max_x = 1
	self.max_y = 1


	#CONVENTION IN THIS GRID
	# -----------------------------------> Y
	# |
	# |
	# |
	# |
	# |
	# |
	# |
	# |
	# |
	# |
	# |
	# | X
	# \/

	#how many x-cells in negative direction (0,0) is 0 in None
	self.x_neg = 0

	#how many y-cells in negative direction (0,0) is 0 in None
	self.y_neg = 0



   
   def get(self,x,y):
	return self.grid[x][y]

   def set(self,x,y, new_occupancy):

	x_diff = max(x - (self.max_x - self.x_neg -1), -x - self.x_neg)
	#find if the X value is out of range
	if x_diff > 0:

		for index in xrange(x_diff):
			self.max_x += 2
			self.x_neg += 1

			#expand in max direction
			self.grid.append([0]*self.max_y)
			self.grid.insert(0, [0]*self.max_y)
			
		
	y_diff = max(y - (self.max_y - self.y_neg -1), -y - self.y_neg)
	if y_diff > 0:
		
		for index in xrange(y_diff):
			self.max_y += 2
			self.y_neg += 1

			#expand along X
			for row in self.grid:

				#expand every row to allow more positive and negative values
				row.insert(0, 0)
				row.append(0)	
	

        self.grid[x+self.x_neg][y+self.y_neg] = new_occupancy
	
	
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

	   #make a pathing grid
	   self.planning_grid = Planning_Grid()
	   

    def update_grid(changed_occupancies):

	   #TODO make the x/y snap if needed
	
	   for occ in changed_occupancies:

		#TODO maybe change occupancy grid resolution for pathing etc
		#TODO check if nest or origin become occupied, which is an error

		#normalizing the indexes to start from 0 to 0 etc
		x = occ[0] 
		y = occ[1] 
		planning_grid.set(x, y, occ[2])

    #add a food source on current grid space
    def add_food_source(self):
	   food_source = Food_Source(self.current_cell)
	   if food_source not in pathing_state.food_sources
	   	self.food_source.append(food_source)
	   

    #check if has uncollected food sources
    def has_uncollected_food(self):
	   for food_source in self.food_sources:
		if not food_source.picked_up:
			return True
	   return False

    #distance between cells
    def cell_distance(self, from_cell, to_cell):
	   x_diff = from_cell.x - to_cell.x
	   y_doff = from_cell.y - to_cell.y
	   return math.sqrt( math.pow(x_diff,2) + math.pow(y_diff,2))

   #get closest uncollected food source to cell passed as argument
   def get_closest_food(self):
	   
	   closest_food = None
	   closest_dist = 0

	   for food_source in self.food_sources:
		if not food_source.picked_up:

			distance = self.cell_distance(self.current_cell, food_source.cell)
			if closest_food is not None or distance <= closest_dist:
				closest_food = food_source

	   return closest_food

    #TODO make sure no food sources on same grid	
    #pick up the food on the spot the command was called if there is an unpicked one
    def pick_food_up(self):
	for food_source in self.food_sources:
		if food_source.location == self.current_cell and not food_source.picked_up:		
			food_source.picked_up = True
			self.food += 1
	   	
    #TODO sync with angus
    # get closest cell dimensions
    def snap(self, value):
	return ( int(value) / constants.CELL_DIMENSION) * constants.CELL_DIMENSION

    #get the cell coordinates 
    def get_cell(self, pose):

	#snap coordinates to closest cell
	x = self.snap(pose[0])
	y = self.snap(pose[1])

	
	for width in xrange(self.pathing_grid.max_x):
		for height in xrange(self.pathing_grid.max_y):
			cell = self.pathing_grid.get(width,height)
			if cell.get_coordinates() == (x,y):
				return cell

        return None


    #drop off food at the nest and finish
    def	drop_off_food(self):

	#indicate we are at the nest
	self.food = 0
	#drop off food
	self.done = True
	#reset food sources
	for food_source in self.food_sources:
		food_source.picked_up = False

    def are_on_food():
	closest_food = self.get_closest_food()
	#return if the closest cell is the one we are on
	return closest_food.location.get_coordinates() == self.current_cell.get_coordinates()
    
