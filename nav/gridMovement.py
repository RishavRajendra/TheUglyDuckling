from grid import Grid
import grassfire, queue

class GridMovement:

	# direction bytes
	fwd = b'\xA0'
	rev = b'\x0A'
	rotl = b'\x00'
	rotr = b'\xAA'
	strl = b'\x22'
	strr = b'\x88'

	# motion bytes
	allmotors = b'\x55'
	right = b'\x05'
	left = b'\x50'
	front = b'\x11'
	rear = b'\x44'
	right45 = b'\x14'
	left45 = b'\x41'

	def __init__(self, grid, queue):
		self.grid = grid
		self.queue = queue
		self.movement = {(0,1): mov_fwd, (0, -1): mov_rev,
						 				(1,0): mov_strr, (-1, 0): mov_strl,
										(1,1): mov_fwdr, (-1, 1): mov_fwdl,
										(1,-1): mov_revr, (-1,-1): mov_revl}

	def mov_fwd(self):
		self.queue.put(['gridMove', (fwd, allmotors)])

	def mov_rev(self):
		self.queue.put(['gridMove', (rev, allmotors)])

	def mov_strr(self):
		self.queue.put(['gridMove', (strr, allmotors)])

	def mov_strl(self):
		self.queue.put(['gridMove', (strl, allmotors)])

	def mov_fwdr(self):
		self.queue.put(['gridMove', (fwd, right45)])

	def mov_fwdl(self):
		self.queue.put(['gridMove', (fwd, left45)])

	def mov_revr(self):
		self.queue.put(['gridMove', (rev, left45)])

	def mov_revl(self):
		self.queue.put(['gridMove', (rev, right45)])