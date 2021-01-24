#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Float32
from std_msgs.msg import UInt8
import numpy as np
import time

class move():
	def __init__(self):

		rospy.init_node('movement')
		self.action = UInt8()
		self.angle_error = None
		self.old_angle_error = None
		# self.pos_kp = None
		self.i_pos = None					#publish variables
		self.i_angle = None
		# self.delta= np.pi/200			#increment variable

		self.lastTime = 0
		self.now = None
		self.timeChange = None
		self.kd = 0.05
		self.kp = 10

		self.movement_sub = rospy.Subscriber('/movement_info',UInt8,self.movement_callback)
		self.angleKp_sub = rospy.Subscriber('/angle_kp',Float32,self.angle_kp_callback)
		# self.pos_kp_sub = rospy.Subscriber('/pos_kp',Float32,self.pos_kp_callback)
		self.leftWheel = rospy.Publisher('/foxbot/left_wheel_controller/command', Float64, queue_size=10)
		self.rightWheel = rospy.Publisher('/foxbot/right_wheel_controller/command', Float64, queue_size=10)

	def movement_callback(self, data):
		self.action = data.data

	def angle_kp_callback(self, data):
		self.old_angle_error = self.angle_error
		self.angle_error = data.data
		# print('angle : '+ str(self.angle_kp))
	# def pos_kp_callback(self, data):
		# self.pos_kp = data.data
		# print('pos : '+str(self.pos_kp))

	def controlwheels(self):
		if self.angle_error is not None and self.old_angle_error is not None and self.action is not None:
			if self.timeChange<=10000:
				# print(self.pos_kp)
				self.i_pos = 5.0
				self.i_angle = self.kp*self.angle_error + (self.angle_error-self.old_angle_error)/self.timeChange*self.kd
				if self.i_angle>2:
					self.i_angle = 2
				if self.action == 1:
					# print("forward")
					self.leftWheel.publish(self.i_pos)
					self.rightWheel.publish(self.i_pos)
				elif self.action == 2:
					# print("left")
					# self.leftWheel.publish(0)
					self.rightWheel.publish(0)
					self.leftWheel.publish(self.i_angle)
				elif self.action == 3:
					# print("right")
					self.leftWheel.publish(0)
					# self.rightWheel.publish(0)
					self.rightWheel.publish(self.i_angle)
				else:
					print("stop")
					self.leftWheel.publish(0)
					self.rightWheel.publish(0)

				
				# print('command : '+str(self.i_angle)+' timeChange : '+str(self.timeChange)+' deltaTheta : '+str(self.angle_error-self.old_angle_error))
		

if __name__ == '__main__':
	m = move()
	r = rospy.Rate(30)
	while not rospy.is_shutdown():
		m.now = time.time()
		m.timeChange = m.now - m.lastTime
		if m.timeChange>=(1/float(30)):
			m.controlwheels()
		r.sleep()
		m.lastTime = m.now
		if m.action==4:
			break

