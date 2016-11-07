#
#     - - - -  I A R  - - - -
#
#  s1311631         Angus Pearson
#  s1346981    Jevgenij Zubovskij
#

from AStar import Cell
import numpy


class Food_Source:
   #make a new structure to indicate if the food was picked up in this round
   def __init__(self, location_cell):
	self.location = location_cell
	self.picked_up = False

class Planning_Grid:
   def __init__(self, max_x, max_y):
	#initially whole grid unoccupied
	#TODO uncomment
	#self.grid = numpy.zeros((max_x+10,max_y+10), dtype=int)
	
	self.max_x = max_x
	self.max_y = max_y
	#TODO remove when done debugging
	#TODO check format and do not forget to change this to be empty

	self.grid =  	[[0, 0, 0, 0, 0, 0],
        		 [0, 0, 1, 0, 0, 0],
        		 [0, 1, 0, 0, 0, 0],
        		 [0, 0, 0, 0, 0, 0],
        		 [0, 0, 0, 0, 0, 0],
        		 [0, 0, 0, 0, 0, 0]]

	#how many x-cells in negative direction (0,0) is 0 in None


	#TODO make it 0 initially
	self.x_neg = 0

	#how many y-cells in negative direction (0,0) is 0 in None

	#TODO make it 0 initially
	self.y_neg = 0

	#TODO make the pathing grid bigger than the occupancy as sensors sparse

   
   def get(self,x,y):
	return self.grid[x][y]

   def set(self,x,y, new_occupancy):
	self.grid[x][y] = new_occupancy

	
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
	   self.planning_grid = Planning_Grid(6,6)

    def update_grid(changed_occupancies):
	   for occ in changed_occupancies:

		#TODO change granularity 
		#normalizing the indexes to start from 0 to 0 etc
		x = occ[0] + planning.grid.x_neg
		y = occ[1] + planning.grid.y_neg
		planning_grid.set(x, y, occ[2])





    
    
    
    
