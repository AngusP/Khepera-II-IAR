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
                        if adj_cell.g > cell.g + self.cell_distance(adj_cell, cell):
                            self.update_cell(adj_cell, cell)
                        else:
                            self.update_cell(adj_cell, cell)
                            # add adj cell to open list
                            heapq.heappush(self.opened, (adj_cell.f, adj_cell))

