import cv2 as cv
import time
import pygame as pg
from threading import Thread

class userinterface(object):

	def __init__(self, video_queue, control_queue,i_q):
		self.video_queue = video_queue
		self.control_queue = control_queue
		self.input_queue = i_q
		self.height = 720
		self.width = 960
		self.stopped = True

	def run(self):
		self.stopped = False
		self._display()

	def _display(self):
		pg.init()

		frame = self.video_queue.get()
		h,w,_ = frame.shape
		screen = pg.display.set_mode((w,h))

		while not self.stopped:

			for event in pg.event.get():
			 	if event.type == pg.KEYDOWN:
			 		self._input(event.key)

			screen.fill([0,0,0])
			frame = self.video_queue.get()
			frame = pg.surfarray.make_surface(frame.swapaxes(0,1))
			screen.blit(frame,(0,0))
			pg.display.update()

		pg.display.quit()
		pg.quit()

	def _input(self, key):
		if key == pg.K_l:
			self.control_queue.put("L")
		elif key == pg.K_t:
			self.control_queue.put("T")
		elif key == pg.K_e:
			self.input_queue.put("E")
		elif key == pg.K_ESCAPE:
			self.control_queue.put("ESCAPE")
			self.input_queue.put("ESCAPE")
			self.end()

	def end(self):
		if not self.stopped:
			self.stopped = True