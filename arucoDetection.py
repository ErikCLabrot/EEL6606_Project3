import math
import cv2 as cv
import numpy as np
from threading import Thread


class detector(object):

	def __init__(self, video_queue):
		self.video_queue = video_queue

		self.mtrx = np.array([[914.88712703, 0, 473.06006785], [0, 910.68258674, 348.11604291], [  0, 0, 1 ]])
		self.dist = np.array([ 2.66546113e-02, -3.89699367e-01,  1.36861296e-03, -1.14203954e-04, 1.04497706e+00])
		self.markerSize = 4.60375 #cm
		self.aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_50)
		self.aruco_params = cv.aruco.DetectorParameters()
		self.detector = cv.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

		self.stopped = True

	def run(self):
		self.stopped = False

		while not self.stopped:
			if not self.video_queue.empty():
				frame = self.video_queue.get()
				frame = cv.cvtColor(frame,cv.COLOR_BGR2RGB)
				self.corners, self.ids, self.rj = self.detector.detectMarkers(frame)
				if len(self.corners) > 0:
					rvecs, tvecs = self.computeTF(self.corners)
					r,_ = cv.Rodrigues(rvecs[0])
					angles = self.rotationMatrixToEulerAngles(r)

	def detect(self):
		if not self.video_queue.empty():
			frame = self.video_queue.get()
			frame = cv.cvtColor(frame,cv.COLOR_BGR2RGB)
			self.corners, self.ids, self.rj = self.detector.detectMarkers(frame)
			if len(self.corners) > 0:
				temp = self.corners
				if len(self.ids) > 1:
					if self.ids[0] == 1:
						temp = self.corners[1]
				else:
					temp = self.corners[0]
				rvecs, tvecs = self.computeTF(temp)
				r,_ = cv.Rodrigues(rvecs[0])
				angles = self.rotationMatrixToEulerAngles(r)
				return angles, tvecs[0], True
			else:
				return 0,0,False
		else:
			return 0,0,False

	def computeTF(self,corners):
		marker_points = np.array([[-self.markerSize / 2, self.markerSize / 2, 0],
						  [self.markerSize / 2, self.markerSize / 2, 0],
						  [self.markerSize / 2, -self.markerSize / 2, 0],
						  [-self.markerSize / 2, -self.markerSize / 2, 0]], dtype=np.float32)

		#"Numpy array slices won't work as input because solvePnP requires contiguous arrays"
		trash = []
		rvecs = []
		tvecs = []
		i = 0
		for c in corners:
			nada, R, t = cv.solvePnP(marker_points, corners[i], self.mtrx, self.dist, False, cv.SOLVEPNP_IPPE_SQUARE)
			rvecs.append(R)
			tvecs.append(t)
			trash.append(nada)

			return rvecs, tvecs
	
	def rotationMatrixToEulerAngles(self,R): 

		sy = math.sqrt(R[0,0] * R[0,0] + R[1,0] * R[1,0])

		singular = sy < 1e-6

		if not singular :
			x = np.arctan2(R[2,1] , R[2,2])
			y = np.arctan2(-R[2,0], sy)
			z = np.arctan2(R[1,0], R[0,0])
		else:
			x = math.atan2(-R[1,2], R[1,1])
			y = math.atan2(-R[2,0], sy)
			z = 0

		return np.array([x, y, z]) * (180/np.pi) # move from radians to degrees

	def end(self):
		if not self.stopped:
			self.stopped = True
