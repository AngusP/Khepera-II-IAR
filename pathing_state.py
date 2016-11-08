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
   def __init__(self):
	#initially whole grid unoccupied
	#TODO uncomment
	#self.grid = numpy.zeros((max_x+10,max_y+10), dtype=int)
	
	#the size of the matrix
	self.max_x = 1
	self.max_y = 1
	#TODO remove when done debugging

	#Only know where we ourselves are
	self.grid =  	[[0]]

	#how many x-cells in negative direction (0,0) is 0 in None


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

	#TODO make it 0 initially
	self.x_neg = 0

	#how many y-cells in negative direction (0,0) is 0 in None

	#TODO make it 0 initially
	self.y_neg = 0

	#TODO make the pathing grid bigger than the occupancy as sensors sparse

   
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
			print("EXPAND X")

			
		
	y_diff = max(y - (self.max_y - self.y_neg -1), -y - self.y_neg)
	if y_diff > 0:
		
		for index in xrange(y_diff):
			self.max_y += 2
			self.y_neg += 1

			print("EXPAND Y")
			#expand along X
			for row in self.grid:

				#expand every row to allow more positive and negative values
				row.insert(0, 0)
				row.append(0)




	print("DIFFERENCES X{} Y{}".format(x_diff, y_diff))
	#normalize coordinates
	print(" {} {} = {} DIMENSIONS {} by {}".format(x+self.x_neg,y+self.y_neg,  new_occupancy, self.max_x, self.max_y))

	#print("{} {} reality".format( len(self.grid[4]) , len(self.grid) ))


	for row in self.grid:
			#TODO check 
			print(row)	
	

        self.grid[x+self.x_neg][y+self.y_neg] = new_occupancy

    #def print():
	
	
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


	   #TODO remove these

	   #self.planning_grid.set(1,1,1)
	   #self.planning_grid.set(-1,-1,1)
	   #self.planning_grid.set(5,5,1)
	   #self.planning_grid.set(3, 3, 0)	
	   #self.planning_grid.set(3, 2,0)
	   #self.planning_grid.set(1, 1,0)
	   #self.planning_grid.set(3, 2,1)	
	   #self.planning_grid.set(2, 2,1)
	   self.planning_grid.set(2, 2,1)
	   self.planning_grid.set(4, 4, 0)
	   self.planning_grid.set(2, 2, 11)

           self.planning_grid.set(0, -2, 11)

	   self.planning_grid.set(-1, 1, 1)
	   self.planning_grid.set(0, 1, 1)
	   self.planning_grid.set(1, 1, 1)
	   self.planning_grid.set(2, 1, 1)

	   print("PLANNING GRID")

	   for row in self.planning_grid.grid:
			#TODO check 
			print(row)

	   

    def update_grid(changed_occupancies):

	   #planning_grid.neg_x =
	
	   for occ in changed_occupancies:

		#TODO add changing granularity 
		#normalizing the indexes to start from 0 to 0 etc
		x = occ[0] 
		y = occ[1] 
		planning_grid.set(x, y, occ[2])





    
    
    
    
