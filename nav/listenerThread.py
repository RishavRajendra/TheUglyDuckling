# Thread that listens for signals from arduino.

import queue, threading
import serial, time

class ListenerThread(threading.Thread):

	# (queue) = Queue()
  # (serial) = serial connection to arduino
  # (lock) = threading.Lock()
	def __init__(self, queue, serial, lock):
		super(ListenerThread, self).__init__()
		self.queue = queue
		self.serial = serial
		self.lock = lock
		self.haslock = False
		self.stoprequest = threading.Event()

	def run(self):
		while not self.stoprequest.isSet():
			if not self.queue.empty():
				self.lock.acquire(True)
				print(' l locked')
				# time.sleep(1)
				flag = self.serial.read()
				self.lock.release()
				print('l released')

	def join(self, timeout=None):
		self.stoprequest.set()
		super(ListenerThread, self).join(timeout)

			
