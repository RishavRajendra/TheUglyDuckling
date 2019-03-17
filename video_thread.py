from picamera.array import PiRGBArray
from picamera import PiCamera
from constants import CAMERA_RESOLUTION, CAMERA_FRAMERATE
import threading, queue, cv2
import numpy as np

class VideoThread(threading.Thread):
	def __init__(self, queue, model):
		super(VideoThread, self).__init__()
		self.queue = queue
		self.model = model
		self.stoprequest = threading.Event()
		
	def run(self):
		camera = PiCamera()
		camera.resolution = CAMERA_RESOLUTION
		camera.framerate = CAMERA_FRAMERATE
		rawCapture = PiRGBArray(camera, size=CAMERA_RESOLUTION)
		while not self.stoprequest.isSet():
			rawCapture.truncate(0)
			
			for frame1 in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
				frame = np.copy(frame1.array)
				result = self.model.predict(frame)
				if self.queue.full():
					self.queue.get()
				self.queue.put(result)
				rawCapture.truncate(0)
			if cv2.waitKey(1) == ord('q'):
				break
		camera.close()
			
	def join(self, timeout=None):
		self.stoprequest.set()
		super(VideoThread, self).join(timeout)
