# Grassfire algorithim used for pathfinding.
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

"""
Change log
    -[0.0.1] Benji
        --- If we want to move to a specific spot we can
        --- include the goal
"""
def construct_path(graph, visited, start, include_goal=False):
	current = start
	path = []
	while True:
		lowest_tile = current
		for next in graph.neighbors(current):
			if next in visited:
				if visited[next] < visited[lowest_tile]:
					lowest_tile = next
		if visited[lowest_tile] == 0 and not include_goal:
			break
		path.append(lowest_tile)
		current = lowest_tile
		if visited[lowest_tile] == 0:
			break
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
