# FPS counter for openCV experiments
import datetime as dt

class FPS:
	def __init__(self):
		self._start = None
		self._stop = None
		self._numFrames = 0
	
	def start(self):
		self._start = dt.datetime.now()
		return self

	def stop(self):
		self._stop = dt.datetime.now()
		return self

	def update(self):
		self._numFrames += 1
		return self

	def elapsed(self):
		return(self._stop - self._start).total_seconds()

	def fps(self):
		return self._numFrames / self.elapsed()	
