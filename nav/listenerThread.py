import queue, threading
import serial, time

class ListenerThread(threading.Thread):

	def __init__(self, queue, serial, lock):
		super(ListenerThread, self).__init__()
		self.queue = queue
		self.serial = serial
		self.lock = lock
		self.haslock = False
		self.stoprequest = threading.Event()

	def run(self):
		while not self.stoprequest.isSet():
			while not self.queue.empty():
				self.lock.acquire(True)
				print(' l locked')
				time.sleep(4)
				flag = self.serial.read()
				self.lock.release()
				print('l released')
			# print(self.serial.in_waiting)
			
			# if self.locked:
			# 	print('yes')
			# 	self.lock.release()
			# 	self.locked = False
			# 	time.sleep(.2)

			# if flag == b'\xff':
			# 	print(flag)
			# 	self.lock.acquire()
			# 	print("no")
			# 	self.haslock = True
			# 	# time.sleep(1)

			# elif flag == b'' and self.haslock:
			# 	print('yes')
			# 	self.lock.release()
			# 	self.locked = False

			# self.serial.reset_input_buffer()

	def join(self, timeout=None):
		self.stoprequest.set()
		super(ListenerThread, self).join(timeout)

			
