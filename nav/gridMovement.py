# GridMovement class 
# Handles all grid based navigation for the robot

from grid import Grid
import grassfire as gf 
import queue, time

class GridMovement:

	def __init__(self, grid, queue):
		# direction bytes
		self.fwd = b'\xA0'
		self.rev = b'\x0A'
		self.rotl = b'\x00'
		self.rotr = b'\xAA'
		self.strl = b'\x22'
		self.strr = b'\x88'

		# motion bytes
		self.allmotors = b'\x55'
		self.right = b'\x05'
		self.left = b'\x50'
		self.front = b'\x11'
		self.rear = b'\x44'
		self.left45 = b'\x14'
		self.right45 = b'\x41'
	
		self.grid = grid
		self.queue = queue
		self.current = (7,0) #Hardcoded should be (4,3)
		self.goal = (7,4) #Hardcoded for now. Should be generated by find_goal()
		self.facing = 0
		self.path = []
		self.movement = {
			(0,1): [self.fwd, self.allmotors, 0], (0, -1): [self.rev, self.allmotors, 180],
			(1,0): [self.strr, self.allmotors, 90], (-1, 0): [self.strl, self.allmotors, -90],
			(1,1): [self.fwd, self.right45, 45], (-1, 1): [self.fwd, self.left45, -45],
			(1,-1): [self.rev, self.left45, 135], (-1,-1): [self.rev, self.right45, -135]
			}

	# Not yet implemented
	# Sets grid.blocks closest block to goal.
	def find_goal(self):
		pass

	# Generates shortest path to goal using grassfire algorithim
	def find_path(self):
		visited = gf.search(self.grid, self.current, self.goal)
		self.path = gf.construct_path(self.grid, visited, self.current)

	# Follows the generated path by subtracting the next location
	# from self.current and using translate_dir() and self.movement
	# to determine the proper movement
	def follow_path(self):
		dist = 12 # Default distance we want to move
		for index, mov in enumerate(self.path):
			currentResult = (mov[0] - self.current[0], mov[1] - self.current[1])
			currentResult = self.translate_dir(currentResult)
			if (index != len(self.path) -1):
				nextMov = self.path[index+1]
				nextResult = (nextMov[0] - mov[0], nextMov[1] - mov[1])
				nextResult = self.translate_dir(nextResult)
				# If next move request is the same as current 
				# increase distance moved
				if (currentResult == nextResult):
					dist = dist +12
					self.current = mov
					continue
			if(currentResult == (0,1) and dist > 12):
				args = (self.movement[currentResult][0], dist)
				self.queue.put(['accelerate', args])
			else:
				args = (self.movement[currentResult][0], dist, self.movement[currentResult][1])
				self.queue.put(['gridMove', args])
			
			self.current = mov
			# reset distance in case there was a stacked call 
			dist = 12
		# face goal after following path
		self.face_goal()

	def face_goal(self):
		result = (self.goal[0] - self.current[0], self.goal[1] - self.current[1])
		result = self.translate_dir(result)
		degrees = self.movement[result][2]
		if( degrees > 0):
			self.queue.put(['turn', (self.rotr, degrees)])
		elif(degrees < 0):
			self.queue.put(['turn', (self.rotl, degrees*-1)])
		self.facing = self.facing + degrees
		self.trim_facing()

	# Should be called anytime facing is updated
	# Keeps facing between -180 and 180 
	def trim_facing(self):
		if (self.facing > 180):
			self.facing = self.facing - 180
		elif (self.facing < -180):
			self.facing = self.facing + 180

	# Use facing to translate proper movement
	def translate_dir(self, mov):
		result = None
		if(self.facing == 0):
			result = mov
		elif(self.facing == 180 or self.facing == -180):
			result = (mov[0] * -1, mov[1] * -1)
		elif(self.facing == -90):
			result = (mov[1], mov[0] * -1)
		elif(self.facing == 90):
			result = (mov[1] * -1, mov[0])

		return result