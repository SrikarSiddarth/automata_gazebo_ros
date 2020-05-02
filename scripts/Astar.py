import cv2
import numpy as np
import heapq

# no of rows and columns in grid
rows = 29
columns = 29

# start_coords = [14, 14]
# end_coords = [2, 23] 

def grid_map(roi, row_width, col_width, start_coords, end_coords):
    # create a 2d array
    # print(row_width)
    grid = np.zeros((rows, columns))
    for i in range(rows):
        for j in range(columns):
            # white blocks
            if roi[ int(2*row_width*i + row_width), int(2*col_width*j + col_width) ] == 255 :
                grid[i][j] = 0
            # obstacles ->black blocks
            else:
                grid[i][j] = 1
    grid[start_coords[1]][start_coords[0]] = 2
    grid[end_coords[1]][end_coords[0]] = 3
    return grid


class Cell(object):
    def __init__(self, x, y, reachable):
        # setting some parameters for each cell
        self.reachable = reachable
        self.x = x
        self.y = y
        self.parent = None
        self.cost = 0
        self.heuristic = 0
        # net_cost=cost+heuristic
        self.net_cost = 0


class Astar(object):
	def __init__(self):
		# list of unchecked neighbour cells
		self.open = []
		# keeps cells with lowest total_cost at top
		heapq.heapify(self.open)
		# list of already checked cells
		self.closed = set()
		# list of neighbour cells
		self.cells = []

	def init_grid(self, grid, rw, cw):
		for i in range(rows):
			for j in range(columns):
				# detecting the obstacles
				# detecting the start and end
				if(grid[i][j] == 2):
					reachable = True
				elif(grid[i][j] == 3):
					reachable = True
				elif (grid[i][j] == 1) or (i >0 and i<rows-1 and grid[i+1][j] ==1)  or (j >0 and j<columns-1 and grid[i][j+1] ==1)  :
					reachable = False
				elif (grid[i][j] == 1) or (i >0 and i<rows-1 and grid[i-1][j] ==1)  or (j >0 and j<columns-1 and grid[i][j-1] ==1) :
					reachable = False
				elif (grid[i][j] == 1) or (j >0 and j<columns-1 and i >0 and i<rows-1 and grid[i-1][j-1] ==1)  or (i >0 and i<rows-1 and j >0 and j<columns-1 and grid[i+1][j-1] ==1) or (j >0 and j<columns-1 and i >0 and i<rows-1 and grid[i-1][j+1] ==1)  or (i >0 and i<rows-1 and j >0 and j<columns-1 and grid[i+1][j+1] ==1) :
					reachable = False
				else:
					reachable = True
				self.cells.append(Cell(i, j, reachable))

				# detecting the start and end
				if(grid[i][j] == 2):
					self.start = self.cell(i, j)
					reachable = True
				if(grid[i][j] == 3):
					self.end = self.cell(i, j)
					print('hi')
					reachable = True

				
				

	def cell(self, x, y):
		# returns the location to identify each cell
		return self.cells[x*rows+y]

	def cell_heuristic(self, cell):
		# returns the heuristic for astar algo
		return abs(cell.x-self.end.x)+abs(cell.y-self.end.y)

	def neighbour(self, cell):
		cells = []
		# returns a list of neigbours of a cell
		if cell.x < columns - 1:
			cells.append(self.cell(cell.x+1, cell.y))
		if cell.x > 0:
			cells.append(self.cell(cell.x-1, cell.y))
		if cell.y < rows-1:
			cells.append(self.cell(cell.x, cell.y+1))
		if cell.y > 0:
			cells.append(self.cell(cell.x, cell.y-1))
		return cells

	def update_cell(self, adj, cell):
		# update the details about the selected neigbour cell
		adj.cost = cell.cost + 1
		adj.heuristic = self.cell_heuristic(adj)
		adj.parent = cell
		adj.net_cost = adj.cost + adj.heuristic

	def display_path(self):
		# list for storing the path
		route_path = []
		# flag to determine length of path
		count = 0
		cell = self.end
		while cell.parent is not None:
			# storing the parents in list from end to start
			route_path.append([(cell.y)+1, (cell.x)+1])
			cell = cell.parent
			count += 1
		return route_path, count

	def search(self):
		# pushing the first element in open queue
		heapq.heappush(self.open, (self.start.net_cost, self.start))
		while(len(self.open)):
			net_cost, cell = heapq.heappop(self.open)
			# adding the checked cell to closed list
			self.closed.add(cell)
			if cell is self.end:
				# store path and path legth
				print('hi2')
				route_path3, route_length = self.display_path()
				route_path3.reverse()
				break
			# getting the adjoint cells
			neighbours = self.neighbour(cell)
			for path in neighbours:
				# if cell is not an obstacle and has not been already checked
				if path.reachable and path not in self.closed:
					if (path.net_cost, path) in self.open:
						# selecting the cell with least cost
						if path.cost > cell.cost + 1:
							self.update_cell(path, cell)
					else:
						self.update_cell(path, cell)
						heapq.heappush(self.open, (path.net_cost, path))
		return route_path3, route_length


def play(img, start_coords, end_coords):
	
	
	_, roi = cv2.threshold(img, 200, 255, cv2.THRESH_BINARY)
	imrows, imcols = roi.shape
	rw = float(imrows)/(2*rows)							#half of row width
	cw = float(imcols)/(2*columns)							#half of column width

	# map the grid in an array
	grid = grid_map(roi, rw, cw, start_coords, end_coords)
	# executing A*
	solution = Astar()
	solution.init_grid(grid, rw, cw)
	route_path, route_length = solution.search()


	print "OUTPUT FOR SINGLE IMAGE (IMAGE 1)..."
	print "route length = ", route_length
	

	finalArray = []
	# code for checking output for all images
	for i in range (route_length) :
		finalArray.append([route_path[i][0]*rw*2 - rw, route_path[i][1]*cw*2-cw])
		#cv2.circle(roi, ((2*route_path[i][0])*row_width, (2*route_path[i][1])*col_width), 3, 125, -1)
		# cv2.circle(roi, (finalArray[i][0], finalArray[i][1]), 3, 125, -1)
		#return finalArray[0]
	# print ("route path   = " + str(finalArray))
	# print(imrows,rw) 
	# cv2.imshow('dsfr',cv2.resize(roi, (0,0), fx=.9, fy=.9))
	# cv2.waitKey()
	# cv2.destroyAllWindows()
	return finalArray

	
	



#	arena 	start 	end
#	2	  (14,14)	(14,28)
#	3	  (14,14)	(5,0)
#	4	  (14,14)	(14,28)
#	5	  (14,14)	()	
