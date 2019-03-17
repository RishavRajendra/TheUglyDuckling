# Grid class
# Holds information related to field of play.
# Takes width and height based on number of squares/tiles not actual measurements

class Grid:

	def __init__(self, width, height):
		self.width = width
		self.height = height
		self.obstacles = []
		self.targets = [] #Not used yet
		self.mothership = [] #Not used yet

	def in_bounds(self, id):
		(x, y) = id
		return 0 <= x < self.width and 0 <= y < self.height

	def passable(self, id):
		return id not in self.obstacles

	def neighbors(self, id):
		(x, y) = id
		results = [(x+1, y), (x-1, y), # E and W
							 (x, y+1), (x, y-1), # N and S
							 (x+1, y+1), (x-1, y+1), # NE and NW
							 (x+1, y-1), (x-1, y-1)] # SE and SW
		results = filter(self.in_bounds, results) # Only coordinates in bounds
		results = filter(self.passable, results) # Only unoccupied coordinates 
		return results

	def add_obstacle(self, obstacle):
		if (obstacle not in self.obstacles):
			self.obstacles.append(obstacle)

	def add_target(self, target):
		if (target not in self.targets):
			self.targets.append(target)
