#
#     - - - -  I A R  - - - -
#
#  s1311631         Angus Pearson
#  s1346981    Jevgenij Zubovskij
#

import heapq
import numpy

class Cell(object):
    #make a new cell
    def __init__(self, x, y, reachable):

        self.reachable = reachable

	#pose X
        self.x = x
	#pose Y
        self.y = y
	#for path calculation
        self.parent = None
        #cost of moving from satrt cell to this cell
        self.g = 0
	#heuristic cost from this cell to end cell
        self.h = 0
	#total cost g+h
        self.f = 0

    #custom print function
    def __str__(self):
        return "({}, {}, {})".format(self.x, self.y, self.reachable)


    #custom equality function
    def __eq__(self, other):
        #reachability should not change during planning, so only X,Y matter for comparison
        x_equal = self.x == other.x
        y_equal = self.y == other.y
        return x_equal and y_equal

    #custom inequality function
    def __ne__(self, other):
        return not self.__eq__(other)

    #return pose tuple    
    def get_coordinates(self):
        return (self.x , self.y)

class AStar(object):
    def __init__(self, planning_granularity, data_getter):

        #granularity in mm at which we plan
        self.granularity = planning_granularity
	#the getter for unknown cells (ds server)
        self.getter = data_getter
	#initialize al lvariables
        self.reset()
        
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

    def reset(self):
        

        #self.grid = numpy.zeros((1,1), dtype=None)


        # open list
        self.opened = []
        heapq.heapify(self.opened)
        # visited cells list
        self.closed = set()

        #only know where we ourselves are and that cell is unoccupied
        self.grid = [[None]]
        
        #the size of the matrix
        self.max_x = 1
        self.max_y = 1

        #how many x-cells in negative direction (0,0) is 0 in None
        self.x_neg = 0
        #how many y-cells in negative direction (0,0) is 0 in None
        self.y_neg = 0

        #global planning start and goal
        self.start = None
        self.end   = None



    #return if the value indicates a threshold where we consider the cell occupid
    def is_occupid(self, value):
	#TODO check
	if value >= 50:
		return True
	return False 

    #snap passed actual X or Y value to a math planning grid granularity
    def snap(self, value):
        return ( int(value) / self.granularity) * self.granularity

    #take an actual X or Y and normalize it to a cell index
    def normalize(self, value):
        #print self.snap(value) / self.granularity
        return self.snap(value) / self.granularity

    #use pose to get the indexes in the current grid which would get us the cell with that pose
    def get_cell_indexes(self, x,y):
        x_index = self.normalize(x) + self.x_neg
        y_index = self.normalize(y) + self.y_neg
        return (x_index, y_index)

    #get cell with the X,Y pose indicated 
    def get_cell(self,x,y):
        #get indexes in the gid (so need to convert to indices)
        x_index, y_index = self.get_cell_indexes(x,y)
       
        #check if have such a cell in memory
        if x_index >= self.max_x or x_index < 0 or y_index >= self.max_y or y_index < 0:
      
	    #get cell and add to local gache
	    #occupancy = self.getter.get(x,y)
	    occupancy = True


	    
	    #TODO uncomment and use
	    #occupied = self.is_occupid(occupancy)
	    self.set_cell(x, y, occupancy)
            return self.get_cell(x,y)
        else:
	    cell = self.grid[x_index][y_index]
	    if cell is None:
		#get cell and add to local gache
	    	#occupancy = self.getter.get(x,y)
		occupancy = True

		#TODO uncomment and use
		#occupied = self.is_occupid(occupancy)
		self.set_cell(x, y, occupancy)

	    return self.grid[x_index][y_index]
    #set the occupancy of the cell for indicated X,Y pose
    def set_cell(self,x,y, reachable):

	#get the indexes it would currently occupy
        x_index, y_index = self.get_cell_indexes(x,y)
	
	#if it is withing range, check if have somnething set
	if not (x_index >= self.max_x or x_index < 0 or y_index >= self.max_y or y_index < 0):
		if self.grid[x_index][y_index] is not None:
			#if have something set, have nothing more to do
			return

        #we want to know if we have said index and if not expand grid
        x_index = self.normalize(x)
        y_index = self.normalize(y)
        
        x_diff = max(x_index - (self.max_x - self.x_neg -1), -x_index - self.x_neg)
        #find if the X value is out of range
        if x_diff > 0:

                for index in xrange(x_diff):
                        self.max_x += 2
                        self.x_neg += 1

                        #expand in Y direction                        
                        self.grid.append([None]*self.max_y)
                        self.grid.insert(0, [None]*self.max_y)



        
        y_diff = max(y_index - (self.max_y - self.y_neg -1), -y_index - self.y_neg)
        if y_diff > 0:
                
                for index in xrange(y_diff):
                        self.max_y += 2
                        self.y_neg += 1

                        #expand along X
                        for row in self.grid:

                                #expand every row to allow more positive and negative values
                                row.insert(0, None)
                                row.append(None)   
       

        #re-normalize indexes
        x_index +=self.x_neg
        y_index += self.y_neg
        
        #print "{} {}".format(x_index, y_index)
        self.grid[x_index][y_index] = Cell(self.snap(x),self.snap(y), reachable)
        #print self.grid


    #prepare the grid for planning
    def init_grid(self, start, end):
    
	#reset variable
        self.reset()
        
        #both start and destination should be reachable
        self.set_cell(start[0], start[1], True)
        self.set_cell(end[0], end[1], True)

	#set path end and start
        self.start = self.get_cell(start[0], start[1])
        self.end   =self.get_cell(end[0], end[1])

    #return H
    def get_heuristic(self, cell):
	#calculate heuristic cost - distance between this cell and the ending cell multiply by 10.
        return 10 * (abs(cell.x - self.end.x) + abs(cell.y - self.end.y))

    
    #get adjacent cells to cell in argument
    def get_adjacent_cells(self, cell):

        cells = []

	#get cells at right angles to current cell (make sure they are not None for one reason or another)
	if True:
            a = self.get_cell(cell.x+self.granularity, cell.y)
	    if a is not None:
            	cells.append(a)

            a = self.get_cell(cell.x, cell.y-self.granularity)
	    if a is not None:
            	cells.append(a)	    

	    a = self.get_cell(cell.x-self.granularity, cell.y)
	    if a is not None:
            	cells.append(a)

	    a = self.get_cell(cell.x, cell.y+self.granularity)
	    if a is not None:
            	cells.append(a)
           

	#get diagonal cells and make sure not to append None ones (which should not hapen)
	if True:

                if self.get_cell(cell.x+self.granularity, cell.y).reachable or self.get_cell(cell.x, cell.y+self.granularity).reachable:
	                a = self.get_cell(cell.x+self.granularity, cell.y+self.granularity)
			if a is not None:
            			cells.append(a)

                #check if not blocked by 2 other ones (the 90 degree neighbours)
                if self.get_cell(cell.x+self.granularity, cell.y).reachable or self.get_cell(cell.x, cell.y-self.granularity).reachable:
			a = self.get_cell(cell.x+self.granularity, cell.y-self.granularity)
			if a is not None:
            			cells.append(a)
                        


                #check if not blocked by 2 other ones (the 90 degree neighbours)
                if self.get_cell(cell.x-self.granularity, cell.y).reachable or self.get_cell(cell.x, cell.y+self.granularity).reachable:
                        a = self.get_cell(cell.x-self.granularity, cell.y+self.granularity)
			if a is not None:
            			cells.append(a)

                #check if not blocked by 2 other ones (the 90 degree neighbours)
                if self.get_cell(cell.x-self.granularity, cell.y).reachable or self.get_cell(cell.x, cell.y-self.granularity).reachable:
                        a = self.get_cell(cell.x-self.granularity, cell.y-self.granularity)
			if a is not None:
            			cells.append(a)
        return cells

    #return the calcualted path
    def get_path(self):

        #print "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^"
        cell = self.end
        path = [Cell(cell.x, cell.y, True)]


        #print cell
        while cell.parent is not self.start:
            cell = cell.parent
            #print cell
            path.append(Cell(cell.x, cell.y, True))   

        cell = self.start
        #print cell
        path.append(Cell(cell.x, cell.y, True))

        path.reverse()
        #print "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^"        
        return path

    #function returning distance between cells
    def cell_distance(self, from_cell, to_cell):
	   #euclidian distance
	   x_diff = from_cell.x - to_cell.x
	   y_diff = from_cell.y - to_cell.y
	   return math.sqrt( math.pow(x_diff,2) + math.pow(y_diff,2))

    #upradte the movement costs
    def update_cell(self, adj, cell):	
        adj.g = cell.g + self.cell_distance(adj, cell)
        adj.h = self.get_heuristic(adj)
        adj.parent = cell
        adj.f = adj.h + adj.g

    #path claculator
    def solve(self):

        # add starting cell to open heap queue
        heapq.heappush(self.opened, (self.start.f, self.start))
        
        while len(self.opened):

            # pop cell from heap queue
            f, cell = heapq.heappop(self.opened)
            # add cell to closed list so we don't process it twice
            self.closed.add(cell)
            # if ending cell, return found path
            if cell is self.end:
                return self.get_path()
            # get adjacent cells for cell
            adj_cells = self.get_adjacent_cells(cell)
            for adj_cell in adj_cells:
                if adj_cell.reachable and adj_cell not in self.closed:
                    if (adj_cell.f, adj_cell) in self.opened:
                        # if adj cell in open list, check if current path better 
			# than the previosu one for this adjacent cell.
                        if adj_cell.g > cell.g + self.cell_distance(adj, cell):
                            self.update_cell(adj_cell, cell)
                    else:
                        self.update_cell(adj_cell, cell)
                        # add adj cell to open list
                        heapq.heappush(self.opened, (adj_cell.f, adj_cell))


    #replan the path from start to end
    def replan(self, start, end):

	#account for calculations if we are already at the destination
	start_indexes = self.get_cell_indexes(start[0], start[1])
	end_indexes   = self.get_cell_indexes(end[0], end[1])

	#if are already at the destination, just return a path containing one node
        if end == start or start_indexes == end_indexes:
                return [Cell(start[0], start[1], True)]

	#initalize the grid
        self.init_grid(start, end)
        #solve the pathing
        self.solve()
	#get the path
        path = self.get_path()

        return path

    #function to conveniently print the grid
    def print_grid(self):
        grid = [[]]
        print "############################################################################"
        for x in xrange(self.max_x):
           line = []
           for y in xrange(self.max_y):
               cell = self.grid[x][y]
               if cell is None:
                   line.append(None)
               else:
                   line.append(cell.reachable)
           print line
           grid.append(line)

        #print grid   
        print "############################################################################"

