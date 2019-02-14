from grid import Grid
from queue import Queue

def search(graph, start, goal):
	unvisited = Queue()
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
				visited[next] = visited[current]+1

	return visited

def construct_path(graph, visited, start):
	current = start
	path = []
	while True:
		lowest_tile = current
		for next in graph.neighbors(current):
			if next in visited:
				if visited[next] < visited[lowest_tile]:
					lowest_tile = next
		if visited[lowest_tile] == 0:
			break
		path.append(lowest_tile)
		current = lowest_tile
	return path