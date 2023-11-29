import time
import cv2 as cv
from threading import Thread
from queue import Queue
from djitellopy import Tello

class tellomanager(object):
	def __init__(self, data_queue, video_queue, control_queue, detection_queue):
		self.data_queue = data_queue
		self.video_queue = video_queue
		self.control_queue = control_queue
		self.detection_queue = detection_queue

		self.data_running = False
		self.video_running = False
		self.control_running = False

		self.tello = Tello()
		self.run = False
		self.running = False
		print("Tello manager initialized")

	def Run(self):
		self.run = True
		self.running = True
		self.tello.connect()
		self.tello.set_speed(10)
		self.tello.streamon()

		print("Beginning communication threads")

		data_loop = Thread(target=self._update_data)
		video_loop = Thread(target=self._update_video)
		control_loop = Thread(target=self._handle_control)

		data_loop.start()
		video_loop.start()
		control_loop.start()

		data_loop.join()
		video_loop.join()
		control_loop.join()

		self.running = False

	def _update_data(self):
		while self.run:
			state = self.tello.get_current_state()
			self.data_queue.put(state)
			time.sleep(1/1000)

	def _update_video(self):
		frame = self.tello.get_frame_read()
		time.sleep(1)
		while self.run:
			img = frame.frame
			self.video_queue.put(img)
			self.detection_queue.put(img)
			time.sleep(1/60)

	def _handle_control(self):
		while self.run:
			if not self.control_queue.empty():
				control = self.control_queue.get()
				if control == "L" and self.tello.is_flying:
					self.tello.land()
				elif control == "T" and not self.tello.is_flying:
					self.tello.takeoff()
				elif control == "ESCAPE":
					self.end()
				elif control == "X":
					self.tello.move_forward(85)
				else:
					self.tello.send_rc_control(control[0],control[1],control[2],control[3])

	def end(self):
		self.run = False

		if self.tello.is_flying:
			self.tello.land()

		while self.running:
			time.sleep(1)

		self.tello.streamoff()
		self.tello.end()