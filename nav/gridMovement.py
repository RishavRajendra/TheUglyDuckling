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
		self.current = (0,0)
		self.goal = (0,6)
		self.facing = 'N'
		self.path = []
		self.movement = {(0,1): self.mov_fwd, (0, -1): self.mov_rev,
						 				(1,0): self.mov_strr, (-1, 0): self.mov_strl,
										(1,1): self.mov_fwdr, (-1, 1): self.mov_fwdl,
										(1,-1): self.mov_revr, (-1,-1): self.mov_revl}

	# Not yet implemented
	# Sets grid.blocks closest block to goal.
	def find_goal(self):
		pass

	# Generates shortest path to goal using grassfire algorithim
	def find_path(self):
		visited = gf.search(self.grid, self.current, self.goal)
		self.path = gf.construct_path(self.grid, visited, self.current)
		print(self.path)

	# Follows the generated path by subtracting the next location
	# from self.current and using translate_dir() and self.movement
	# to determine the proper movement
	def follow_path(self):
		for mov in self.path:
			result = (mov[0] - self.current[0], mov[1] - self.current[1])
			# print(result)
			result = self.translate_dir(result)
			# print(result)
			# print(self.movement[result])
			self.movement[result]()
			self.current = mov
			# time.sleep(2)
		self.queue.put(['turn', (b'\x00', 45)])

	# Use facing to translate proper movement
	def translate_dir(self, mov):
		result = None
		if(self.facing == 'N'):
			result = mov
		elif(self.facing == 'S'):
			result = (mov[0] * -1, mov[1] * -1)
		elif(self.facing == 'W'):
			result = (mov[1], mov[0] * -1)
		elif(self.facing == 'E'):
			result = (mov[1] * -1, mov[0])

		return result

	# Movement functions stored in movement dict 
	def mov_fwd(self):
		self.queue.put(['gridMove', (self.fwd, self.allmotors)], True, 0.05)

	def mov_rev(self):
		self.queue.put(['gridMove', (self.rev, self.allmotors)], True, 0.05)

	def mov_strr(self):
		self.queue.put(['gridMove', (self.strr, self.allmotors)], True, 0.05)

	def mov_strl(self):
		self.queue.put(['gridMove', (self.strl, self.allmotors)], True, 0.05)

	def mov_fwdr(self):
		self.queue.put(['gridMove', (self.fwd, self.right45)], True, 0.05)

	def mov_fwdl(self):
		self.queue.put(['gridMove', (self.fwd, self.left45)], True, 0.05)

	def mov_revr(self):
		self.queue.put(['gridMove', (self.rev, self.left45)], True, 0.05)

	def mov_revl(self):
		self.queue.put(['gridMove', (self.rev, self.right45)], True, 0.05)