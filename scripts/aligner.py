#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Point32
from std_msgs.msg import Float32
from std_msgs.msg import UInt8


class Align():
	"""class Align"""
	def __init__(self):
		rospy.init_node('aligner')
		self.action = UInt8()
		self.coords = Point32()
		self.angle = None
		self.action = 0
		self.move_pub = rospy.Publisher("/movement_info",UInt8,queue_size=10)
		self.angle_kp_pub = rospy.Publisher("/angle_kp",Float32,queue_size=10)
		self.pos_kp_pub = rospy.Publisher("/pos_kp",Float32,queue_size=10)
		self.angle_sub = rospy.Subscriber('/direction_info',Float32,self.angle_callback)
		self.pos_sub = rospy.Subscriber('/position_info',Point32,self.pos_callback)
		self.finalArray = None
		self.i = 0						#iterating variable for points on the shortest path
		self.angle2 = None
		self.angle_difference = None
		self.displacement = None
		self.zone = 0

	def angle_callback(self, data):
		self.angle = data.data
	def pos_callback(self, data):
		self.coords.x = data.x
		self.coords.y = data.y

	def load_array(self):
		self.finalArray = np.load("/home/phani/catkin_ws/src/automata/scripts/finalArray.npy")
		# print(self.finalArray)

	def run(self):
		if self.coords is not None and self.angle is not None:
			num = self.finalArray[self.i][1]-self.coords.y
			den = self.finalArray[self.i][0]-self.coords.x
			self.displacement = np.sqrt(np.power(num,2)+np.power(den,2))
			self.angle2 = np.arctan2(num,den)*(180/np.pi)
			self.del_theta = self.angle-self.angle2			#angle_difference
			print('linear_displacement : ' + str(self.displacement) + ' i : ' + str(self.i) + ' angular_displacement : ' + str(self.del_theta) + ' action : '+str(self.action))
			
			
			
			
			if self.displacement<10:						#keeping shifting distance to 30 units
				self.zone = 2
			elif abs(self.del_theta)<1:						#keeping the angle error to '3' degrees
				self.zone = 1
			else:
				self.zone = 0
			# print('zone : '+ str(self.zone))

			if self.zone==0:
				if (self.del_theta>0 and self.del_theta<180) or self.del_theta<-180:
					self.action = 2							#go left
				elif (self.del_theta<0 and self.del_theta>-180) or self.del_theta>180:
					self.action = 3							#go right
			elif self.zone==1:
				self.action = 1								#go forward
			else:
				self.i +=1
				if self.i == len(self.finalArray):
					self.action = 4							#stop
					print('Hara Hara Maha Deva! The bot has completed the path and reached the destination!')
			self.move_pub.publish(self.action)
			self.angle_kp_pub.publish(abs(self.del_theta/360))
			self.pos_kp_pub.publish(self.displacement/1000)







if __name__ == '__main__':
	try:
		a = Align()
		r = rospy.Rate(30)
		a.load_array()
	except KeyboardInterrupt:
		print('exiting on KeyboardInterrupt')
		exit()
	while not rospy.is_shutdown() and a.action!=4:
		a.run()
		r.sleep()
	
