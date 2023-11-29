import math
import time
import numpy as np
import cv2 as cv
import threading
from arucoDetection import detector

class PID(object):
	def __init__(self, kp, ki, kd):
		self.kp = kp
		self.ki = ki
		self.kd = kd

		self.prevError = 0
		self.intError = 0
		self.stopped = False

	def control(self, error):
		self.intError += error
		if self.prevError == 0:
			self.prevError = error
		dError = error - self.prevError
		self.prevError = error
		output = self.kp*error + self.ki*self.intError + self.kd*dError
		return output

	def reset(self):
		self.prevError = 0
		self.intError = 0

class controller(object):
	def __init__(self, sensor_queue, control_queue,input_queue):
		self.sensor_queue = sensor_queue
		self.control_queue = control_queue
		self.input_queue = input_queue
		self.aruco = detector(self.sensor_queue)
		self.apid = PID(1.2,0.00001,0.005) #East
		self.bpid = PID(1.2,0.00001,0.005) #North
		self.cpid = PID(1.2,0.00001,0.005) #Up
		self.dpid = PID(0.5,0.00001,0.005) #Yaw

		self.set_point = np.array([0,10,75,0]) #EUNY
		self.current_err = np.array([0,0,0,0])
		self.max_speed = 100

		self.stopped = False
		self.idling = True
		self.navFlag = False
		self.navigating = False
		self.shouldJoin = False

	def run(self):
		while self.stopped:
			self.checkInput()
		self._run()

	def _run(self):
		navThread = threading.Thread(target=self.navigationTimer)
		navThread.start()
		while not self.stopped:
			angles,tvec,detected = self.aruco.detect()

			if detected:
				self.idling = False

			if not detected and not self.idling:
				self.idling = True
				self.control_queue.put([0,0,0,0])

			if detected and not self.navigating:
				yaw = angles[1]
				self.updateControl(yaw,tvec)
				self.navigateObstacle()
				self.checkInput()

			time.sleep(1/60)

	def updateControl(self, yaw, tvecs):
		#get up to date detection
		north_err = tvecs[2] - self.set_point[2]
		east_err = tvecs[0] - self.set_point[0]
		up_err = -(tvecs[1] - self.set_point[1])
		yaw_err = yaw - self.set_point[3]
		self.current_err = np.array([north_err, east_err, up_err, yaw_err])
		#calculate inputs
		a = self.apid.control(east_err)
		b = self.bpid.control(north_err)
		c = self.cpid.control(up_err)
		d = self.dpid.control(yaw_err)
		
		ms = int(self.max_speed)
		amp = 1
		#Range fit inputs
		a = int(np.clip(amp*a,-ms,ms))
		b = int(np.clip(b,-ms,ms))
		c = int(np.clip(amp*c,-ms,ms))
		d = int(np.clip(d,-ms,ms))
		print("err")
		print(self.current_err)
		print("inp")
		print([a,b,c,d])
		#send inputs
		self.control_queue.put([a,b,c,d])

	def navigateObstacle(self):
		n_radius = 10
		e_radius = 3
		u_radius = 5
		yaw_radius = 5
		set_point_radius_reached = abs(self.current_err[0]) < n_radius and abs(self.current_err[1]) < e_radius and abs(self.current_err[2]) < u_radius and abs(self.current_err[3]) < yaw_radius
		if set_point_radius_reached:
			time.sleep(1/100000)
			self.control_queue.put("X")
			self.navFlag = True

	def checkInput(self):
		if not self.input_queue.empty():
			inp = self.input_queue.get()
			if inp == "ESCAPE":
				self.end()
			if inp == "E":
				if self.stopped:
					self.stopped = False

	def navigationTimer(self):
		while True:
			if self.navFlag:
				self.control_queue.put([0,0,0,0])
				self.navigating = True
				time.sleep(5)
				self.navFlag = False
				self.navigating = False
			else:
				time.sleep(1/1000)

	def end(self):
		self.stopped = True
