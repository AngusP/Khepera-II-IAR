#
#     - - - -  I A R  - - - -
#
#  s1311631         Angus Pearson
#  s1346981    Jevgenij Zubovskij
#

from AStar import Cell


class Food_Source:
   #make a new structure to indicate if the food was picked up in this round
   def __init__(self, location_cell):
	self.location = location_cell
	self.picked_up = False
	

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

	   #TODO check format and do not forget to change this to be empty
	   self.grid =  [[0, 0, 0, 0, 0, 1],
        		 [1, 1, 0, 0, 0, 1],
        		 [0, 0, 0, 1, 0, 0],
        		 [0, 1, 1, 0, 0, 1],
        		 [0, 1, 0, 0, 1, 0],
        		 [0, 1, 0, 0, 0, 0]]



    
    
    
    
