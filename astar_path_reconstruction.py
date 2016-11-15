        cell = self.end
        path = [Cell(cell.x, cell.y, True)]

        while cell.parent is not self.start:
            cell = cell.parent
            path.append(Cell(cell.x, cell.y, True))   

        cell = self.start
        path.append(Cell(cell.x, cell.y, True))

        path.reverse()