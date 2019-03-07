# Grassfire algorithim used for pathfinding.

from grid import Grid
import collections, copy, time


def search(graph, start, goal):
	unvisited = MyQueue()
	unvisited.put(goal)
	visited = {}
	visited[goal] = 0

	while not unvisited.empty():
		current = unvisited.get()

		if current == start:
			break
	
		for next in graph.neighbors(current):
			if next not in visited:
				unvisited.put(next)
				# Check if next is diagonal
				# result = (current[0]- next[0], current[1] - next[1])
				visited[next] = visited[current]+1

	return visited

def construct_path(graph, visited, start):
	current = start
	path = []
	# Weight to be added to movements in order to
	# discourage them.
	weight = 1.5
	while True:

		lowest_tile = current
		
		for next_tile in graph.neighbors(current):
			lowest_tile_value = visited[lowest_tile] * 1
			if (lowest_tile is current):
				#Ensure we never stay in place
				# We add weight to current tile so that staying in
				# place is never better than moving diagonal 
				lowest_tile_value = lowest_tile_value + weight

			if next_tile in visited:
				next_value = visited[next_tile] * 1
				# Check if next is diagonal. If it is add weight
				if ( is_diagonal(current, next_tile)):
					x = next_tile[0] - current[0]
					y = next_tile[1] - current[1]
					if ((x,0) in graph.obstacles or (0,y) in graph.obstacles):
						next_value = next_value + weight 
				
				# Check if next_tile is better move call than
				# current lowest_tile
				if next_value < lowest_tile_value:
					lowest_tile = next_tile
		
		# If lowest_tile is 0 then we've reached the goal
		# and we're done
		if visited[lowest_tile] == 0:
			break

		# remove current from visited list so we don't bother checking it
		# and can't possibly move backwards
		del visited[current]
		path.append(lowest_tile)
		current = lowest_tile
	return path

# Checks two connected points to see if they are diagonal
def is_diagonal(current, next):
	result = (current[0] - next[0], current[1] - next[1])
	if (result[0] is not 0 and result[1] is not 0):
		return True
	return False

class MyQueue:
	def __init__(self):
		self.elements = collections.deque()

	def empty(self):
		return len(self.elements) == 0

	def put(self, x):
		self.elements.append(x)

	def get(self):
		return self.elements.popleft()
