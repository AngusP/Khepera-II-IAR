#
#     - - - -  I A R  - - - -
#
#  s1311631         Angus Pearson
#  s1346981    Jevgenij Zubovskij
#

import heapq
#import numpy

class Cell(object):
    def __init__(self, x, y, reachable):
        """Initialize new cell.
        @param reachable is cell reachable? not a wall?
        @param x cell x coordinate
        @param y cell y coordinate
        @param g cost to move from the starting cell to this cell.
        @param h estimation of the cost to move from this cell
                 to the ending cell.
        @param f f = g + h
        """
        self.reachable = reachable
        self.x = x
        self.y = y
        self.parent = None
        self.g = 0
        self.h = 0
        self.f = 0

    def __str__(self):
        return "({}, {}, {})".format(self.x, self.y, self.reachable)


    def __eq__(self, other):
        #reachability should not change during planning, so only X,Y matter for comparison
        x_equal = self.x == other.x
        y_equal = self.y == other.y
        return x_equal and y_equal

    def __ne__(self, other):
        return not self.__eq__(other)

    #TODO note that now we use actual frigging X, Y and granularity


class AStar(object):
    def __init__(self, planning_granularity, data_getter):

        #granularity in mm at which we plan
        self.granularity = planning_granularity
        self.getter = data_getter
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




    #TODO remmber that 0,0 will always be at 0,0

    def reset(self):
        
        #Only know where we ourselves are and that cell is unoccupied
        #self.grid = numpy.zeros((1,1), dtype=int)


        # open list
        self.opened = []
        heapq.heapify(self.opened)
        # visited cells list
        self.closed = set()

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




        

    #snap passed actual X or Y value to a math planning grid granularity
    def snap(self, value):
        return ( int(value) / self.granularity) * self.granularity

    #take an actual X or Y and normalize it to a cell index
    def normalize(self, value):
        #print self.snap(value) / self.granularity
        return self.snap(value) / self.granularity

    def get_cell_indexes(self, x,y):
        x_index = self.normalize(x) + self.x_neg
        y_index = self.normalize(y) + self.y_neg
        return (x_index, y_index)

    #note that the arguments are actual X,Y
    def get_cell(self,x,y):
        #get indexes in the gid (so need to convert to indices)
        x_index, y_index = self.get_cell_indexes(x,y)


       
        #check if have such a cell in memory
        if x_index >= self.max_x or y_index >= self.max_y:
            #TODO implement
            print "{} {} NONEXISTENT access".format(x_index, y_index)
            return None

            #TODO uncomment
            #return self.getter.get(x,y)
        else:
            return self.grid[x_index][y_index]

    #note that the arguments are actual X,Y
    def set_cell(self,x,y, reachable):


        #if already have a cell there, do not add again
        if self.get_cell(x,y) is not None:
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
        

        #print self.grid

        #re-normalize indexes
        x_index +=self.x_neg
        y_index += self.y_neg
        
        #print "{} {}".format(x_index, y_index)
        self.grid[x_index][y_index] = Cell(self.snap(x),self.snap(y), reachable)
        #print self.grid


    def init_grid(self, start, end):
    
        """Prepare grid cells, walls.
        @param start grid starting point x,y tuple.
        @param end grid ending point x,y tuple.
        """

        self.reset()
        
        #both start and destination should be reachable
        self.set_cell(start[0], start[1], True)
        self.set_cell(end[0], end[1], True)

        self.start = self.get_cell(start[0], start[1])
        self.end   =self.get_cell(end[0], end[1])

    def get_heuristic(self, cell):
        """Compute the heuristic value H for a cell.
        Distance between this cell and the ending cell multiply by 10.
        @returns heuristic value H
        """
        return 10 * (abs(cell.x - self.end.x) + abs(cell.y - self.end.y))

    

    def get_adjacent_cells(self, cell):
        """Returns adjacent cells to a cell.
        Clockwise starting from the one on the right.
        @param cell get adjacent cells for this cell
        @returns adjacent cells list.
        """
        cells = []

        x,y = self.get_cell_indexes( cell.x, cell.y)


        #using self.granularity instead of 1 as we are using granularity (which is in actual mm)
        if x < self.max_x-1:
            cells.append(self.get_cell(cell.x+self.granularity, cell.y))
        if y > 0:
            cells.append(self.get_cell(cell.x, cell.y-self.granularity))
        if x > 0:
            cells.append(self.get_cell(cell.x-self.granularity, cell.y))
        if y < self.max_y-1:
            cells.append(self.get_cell(cell.x, cell.y+self.granularity))


        if x < self.max_x-1:
            #check if have diagonals on the left
            
            if y < self.max_y-1:
                #check if not blocked by 2 other ones (the 90 degree neighbours)
                if self.get_cell(cell.x+self.granularity, cell.y).reachable or self.get_cell(cell.x, cell.y+self.granularity).reachable:
                        cells.append(self.get_cell(cell.x+self.granularity, cell.y+self.granularity))

            if y > 0:
                #check if not blocked by 2 other ones (the 90 degree neighbours)
                if self.get_cell(cell.x+self.granularity, cell.y).reachable or self.get_cell(cell.x, cell.y-self.granularity).reachable:
                        cells.append(self.get_cell(cell.x+self.granularity, cell.y-self.granularity))
                        
                
        if cell.x > 0:
            #check if have diagonals on the left                
            if cell.y < self.max_y-1:

                #check if not blocked by 2 other ones (the 90 degree neighbours)
                if self.get_cell(cell.x-self.granularity, cell.y).reachable or self.get_cell(cell.x, cell.y+self.granularity).reachable:
                        cells.append(self.get_cell(cell.x-self.granularity, cell.y+self.granularity))

            if cell.y > 0:
                #check if not blocked by 2 other ones (the 90 degree neighbours)
                if self.get_cell(cell.x-self.granularity, cell.y).reachable or self.get_cell(cell.x, cell.y-self.granularity).reachable:
                        cells.append(self.get_cell(cell.x-self.granularity, cell.y-self.granularity))

        #print cells 
        return cells

    def get_path(self):

        #TODO remove
        x_neg = 0
        y_neg = 0

        print "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^"
        cell = self.end
        path = [Cell(cell.x - x_neg, cell.y - y_neg, True)]


        print cell
        while cell.parent is not self.start:
            cell = cell.parent
            print cell
            path.append(Cell(cell.x - x_neg, cell.y - y_neg, True))   

        cell = self.start
        print cell
        path.append(Cell(cell.x - x_neg, cell.y - y_neg, True))

        path.reverse()
        print "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^"        
        return path

    def update_cell(self, adj, cell):
        """Update adjacent cell.
        @param adj adjacent cell to current cell
        @param cell current cell being processed
        """
        adj.g = cell.g + 10
        adj.h = self.get_heuristic(adj)
        adj.parent = cell
        adj.f = adj.h + adj.g

    def solve(self):
        """Solve maze, find path to ending cell.
        @returns path or None if not found.
        """
        # add starting cell to open heap queue
        heapq.heappush(self.opened, (self.start.f, self.start))

        iteration = 0
        
        while len(self.opened):
            print "ITERATION {}".format(iteration)
            iteration +=1
            
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
                        # if adj cell in open list, check if current path is
                        # better than the one previously found
                        # for this adj cell.
                        if adj_cell.g > cell.g + 10:
                            self.update_cell(adj_cell, cell)
                    else:
                        self.update_cell(adj_cell, cell)
                        # add adj cell to open list
                        heapq.heappush(self.opened, (adj_cell.f, adj_cell))

    #TODO rename grid state to pathing_state
    def replan(self, start, end):    
        if end == start:
                return [Cell(start[0], start[1], True)]

        self.init_grid(start, end)


        #TODO remove this
        for x in xrange(81):
            for y in xrange(81):
                self.set_cell(x, y, True)
                self.set_cell(-x,-y,True)
                self.set_cell(-x,y,True)
                self.set_cell(x,-y,True)

        print self.get_cell(80,80)        

        print self.print_grid()        
        
        self.solve()
        path = self.get_path()

        return path

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

def main():

    a = AStar(10, None)




    #x_index, y_index = a.get_cell_idnexes(0,0)

    #print a.print_grid()

    print a.replan((-10,-10), (80,0))


    #print a.get_cell(30,20)
    

if __name__ == "__main__":
    main()
    
