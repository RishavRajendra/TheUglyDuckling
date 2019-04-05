# Grid class
# Holds information related to field of play.
# Takes width and height based on number of squares/tiles not actual measurements

class Grid:

	def __init__(self, width, height):
		self.width = width
		self.height = height

		# Blocked tiles
		self.obstacles = []
		self.targets = [] 
		self.mothership = []
		self.sides = []
		self.slopes = []
		self.corners = [(0,0),(0,7),(7,0), (7,7)]

		# Potential align points
		self.edges = [(0,1), (0,2), (0,3), (0,4),(0,5), (0,6),
									(1,0), (2,0), (3,0), (4,0),(5,0), (6,0),
									(7,1), (7,2), (7,3), (7,4),(7,5), (7,6),
									(1,7), (2,7), (3,7), (4,7),(5,7), (6,7)]
		
		self.exclusion_list = []
		
		self.obstacles_max = 20
		self.last_side_angle = 0

	def in_bounds(self, id):
		(x, y) = id
		return 0 <= x < self.width and 0 <= y < self.height

	def passable(self, id):
		blocked = []
		blocked.extend(self.obstacles)
		blocked.extend(self.targets)
		blocked.extend(self.corners)
		blocked.extend(self.sides)
		blocked.extend(self.slopes)
		blocked.extend(self.targets)
		blocked.extend(self.mothership)
		return (id not in blocked) or (id in self.exclusion_list)
			

	"""
	Change Log
		[0.0.1] Benji
			--- removed diagonal neighbors to discourage diagonal paths in grassfire 
		[0.0.2] Benji
			--- allow diagonal neighbors if flag is True
	"""
	def neighbors(self, id, diag=False):
		(x, y) = id
		results = 	[(x+1, y), (x-1, y), # E and W
							 	(x, y+1), (x, y-1)] # N and S
							 
		d_results =	[(x+1, y+1), (x-1, y+1), # NE and NW
							 	(x+1, y-1), (x-1, y-1)] # SE and SW

		if diag:
			results.extend(d_results)
		results = filter(self.in_bounds, results) # Only coordinates in bounds
		results = filter(self.passable, results) # Only unoccupied coordinates 
		return results

	def remove_edge(self, edge):
		self.edges.remove(edge)

	def set_obstacles_max(self):
		self.obstacles_max = (len(self.targets)/2)*5

	def add_obstacle(self, obstacle):
		if (obstacle not in self.obstacles):
			if len(self.obstacles) >= self.obstacles_max:
				self.obstacles.pop(0)
			self.obstacles.append(obstacle)
		if obstacle in self.edges:
			self.remove_edge(obstacle)

	def add_target(self, target):
		if (target not in self.targets):
			self.targets.append(target)
		if target in self.edges:
			self.remove_edge(target)

	def add_mothership(self, part):
		if part not in self.mothership:
			self.mothership.append(part)
		if part in self.edges:
			self.remove_edge(part)

	def add_slope(self, slope):
		if slope not in self.slopes:
			self.slopes.append(slope)
		if slope in self.edges:
			self.remove_edge(slope)

	def add_side(self, side):
		if side not in self.sides:
			self.sides.append(side)		
		if side in self.edges:
			self.remove_edge(side)

	def get_mothership(self):
		return self.mothership
	
	def get_obstacles(self):
		return self.obstacles
		

