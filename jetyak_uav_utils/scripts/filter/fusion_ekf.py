import numpy as np
from kalman_filter import KalmanFilter
from data_point import DataPoint

class FusionEKF:
	"""
		Gets inputs from multiple sources and uses a Kalman Filter to estimate the state of the system
		
		The state of the system is X = [x y z xdot ydot zdot] the position and velocity of the tag in the
		drone's body frame. Model is assuming constant acceleration.
		
		The inputs are:
			- IMU Accelerometer data.
			- Tag position data. Tag position in the drone's body frame
	"""
	
	def __init__(self, n, F, P, Htag, Himu, Rtag, Rimu, N):
		self.kalmanF   = KalmanFilter(n)
		self.F         = F
		self.P         = P
		self.Htag      = Htag
		self.Himu      = Himu
		self.Rtag      = Rtag
		self.Rimu      = Rimu
		self.N         = N
		self.timeStamp = None
		self.isInit    = False

	def initialize(self, dataPoint):
		if dataPoint.getID() == 'tagPose':
			self.timeStamp = dataPoint.getTime()
			initialState = np.append(dataPoint.getZ(), [[0], [0], [0], [0], [0], [0]], axis = 0)
			self.kalmanF.initialize(initialState, self.F, self.P, self.N)
			self.isInit = True
			print("Filter initialized")

	def process(self, dataPoint):
		if not self.isInit:
			self.initialize(dataPoint)
		else:
			# Check the data ID
			dt = dataPoint.getTime() - self.timeStamp
			self.timeStamp = dataPoint.getTime()
			
			# Update F and Q Matrices
			self.kalmanF.updateF(dt)
			self.kalmanF.updateQ(dt)
			
			# KF Prediction Step
			self.kalmanF.predict()
			
			# KF Correction Step
			if dataPoint.getID() == 'tagPose':
				#self.kalmanF.predict()
				self.kalmanF.correct(dataPoint.getZ(), self.Htag, self.Rtag)
			elif dataPoint.getID() == 'imuAcc':
				self.kalmanF.correct(dataPoint.getZ(), self.Himu, self.Rimu)
			else:
				print("ID bad")

	def getState(self):
		return self.kalmanF.getState()