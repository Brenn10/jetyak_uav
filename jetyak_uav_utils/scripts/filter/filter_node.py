#!/usr/bin/python

import numpy as np
import rospy as rp
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from data_point import DataPoint

from fusion_ekf import FusionEKF

class FilterNode():

	def __init__(self,):
		rp.init_node("tag_pose_filter")
		self.gravityConst = 9.831
		# Number of States
		self.n = 9

		# Initial State Transition Matrix
		self.F = np.asmatrix(np.eye(self.n))

		# Process Matrix
		self.P = np.asmatrix(1.0e4 * np.eye(self.n))

		# Transition Matrix for Tag measurements
		self.Htag = np.matrix([[1.0, 0, 0, 0, 0, 0, 0, 0, 0],
							   [0, 1.0, 0, 0, 0, 0, 0, 0, 0],
							   [0, 0, 1.0, 0, 0, 0, 0, 0, 0]])

		# Covariance Matrix for Tag measurements
		self.Rtag = np.matrix([[1.0e-6, 0, 0],
							   [0, 1.0e-6, 0],
							   [0, 0, 1.0e-6]])

		# Transition Matrix for IMU measurements
		self.Himu = np.matrix([[0, 0, 0, 0, 0, 0, 1.0, 0, 0],
							   [0, 0, 0, 0, 0, 0, 0, 1.0, 0],
							   [0, 0, 0, 0, 0, 0, 0, 0, 1.0]])

		# Covariance Matrix for IMU measurements
		self.Rimu = np.matrix([[1.0e-3, 0, 0],
							   [0, 1.0e-3, 0],
							   [0, 0, 1.0e-3]])

		# Process Noise Level
		self.N = 1.0

		# Initialize Kalman Filter
		self.fusionF = FusionEKF(self.n, self.F, self.P, self.Htag, self.Himu, self.Rtag, self.Rimu, self.N)

		self.imu_sub = rp.Subscriber("/dji_sdk/imu", Imu, self.imu_callback)
		self.tag_sub = rp.Subscriber("/jetyak_uav_vision/tag_pose", PoseStamped, self.tag_callback)

		self.tagVel_pub = rp.Publisher("/jetyak_uav_vision/tag_velocity", Vector3Stamped, queue_size = 1)
		self.tag_pub = rp.Publisher("/jetyak_uav_vision/filtered_tag", PoseStamped, queue_size = 1)
		rp.spin()

	def tag_callback(self, msg):
		tg = DataPoint()
		tg.setID('tagPose')
		tg.setZ(np.matrix([[msg.pose.position.x], [msg.pose.position.y], [msg.pose.position.z]]))
		tg.setTime(msg.header.stamp.to_sec())
		self.fusionF.process(tg)
		
		c = self.fusionF.getState()
		pubMsg = PoseStamped()
		pubMsg.header.stamp = msg.header.stamp
		pubMsg.pose.position.x= c.item(0)
		pubMsg.pose.position.y= c.item(1)
		pubMsg.pose.position.z= c.item(2)
		pubMsg.pose.orientation = msg.pose.orientation
		self.tag_pub.publish(pubMsg)

		velMsg = Vector3Stamped()
		velMsg.header.stamp = msg.header.stamp
		velMsg.vector.x = c.item(3)
		velMsg.vector.y = c.item(4)
		velMsg.vector.z = c.item(5)
		self.tagVel_pub.publish(velMsg)
		

	def imu_callback(self, msg):
		tg = DataPoint()
		tg.setID('imuAcc')
		tg.setZ(np.matrix([[msg.linear_acceleration.x], [msg.linear_acceleration.y], [msg.linear_acceleration.z - self.gravityConst]]))
		tg.setTime(msg.header.stamp.to_sec())
		self.fusionF.process(tg)

filtered = FilterNode()
