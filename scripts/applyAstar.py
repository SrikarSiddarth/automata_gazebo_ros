#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import cv2
import numpy as np
import Astar
from geometry_msgs.msg import Point32
from std_msgs.msg import Float32
from std_msgs.msg import UInt8

class applyAstar():
	"""docstring for applyAstar"""
	def __init__(self):
		rospy.init_node('applyAstar')
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/gazebo/overhead_cam/image",Image,self.image_callback)
		self.img=None
		self.res=None
		self.flag = 0
		self.flag2 = 0
		self.finalArray = None
		self.action = UInt8()
		self.coords = Point32()
		self.angle = None
		self.action = 0
		self.i = 0						#iterating variable for points on the shortest path
		self.angle2 = None
		self.angle_difference = None
		self.displacement = None
		self.zone = 0


		self.move_pub = rospy.Publisher("/movement_info",UInt8,queue_size=10)
		self.angle_kp_pub = rospy.Publisher("/angle_kp",Float32,queue_size=10)
		self.pos_kp_pub = rospy.Publisher("/pos_kp",Float32,queue_size=10)


		self.angle_sub = rospy.Subscriber('/direction_info',Float32,self.angle_callback)
		self.pos_sub = rospy.Subscriber('/position_info',Point32,self.pos_callback)
		

	def image_callback(self, data):
		try:
			rec = self.bridge.imgmsg_to_cv2(data, "mono16")
			# print(rec.shape)
			#slicing the image
			
			# rec2 = rec[20:1275, 30:1215]				#this is for arena.jpg

			height = 720
			width = 720
			dim = (height, width)
			self.res = cv2.resize(rec, dim, interpolation = cv2.INTER_LINEAR)

			if self.flag == 0 and self.res is not None:
				self.img = np.copy(self.res)
				self.flag = 1			
			
		except CvBridgeError as e:
 			print(e)


 	def angle_callback(self, data):
		self.angle = data.data
	def pos_callback(self, data):
		self.coords.x = data.x
		self.coords.y = data.y		

 	def getPath(self):
 		if self.img is not None:
 			# arena3
 			# start_coords = [14, 14]
 			# end_coords = [5, 0]
 			# arena5
 			start_coords = [14,14]
 			end_coords = [0,24]
			self.img = self.img.astype(np.uint8)
 			self.finalArray = Astar.play(self.img, start_coords, end_coords)
	 		for i in range(len(self.finalArray)):
	 			cv2.circle(self.img, (int(self.finalArray[i][0]), int(self.finalArray[i][1])), 3, 125, -1)
			cv2.imshow('d', self.img)
			cv2.waitKey(2000)	
			np.save('/home/phani/catkin_ws/src/automata/scripts/finalArray', self.finalArray)
			# print(self.finalArray)
			print('finalArray saved.')
	
	def align(self):
		if self.coords is not None and self.angle is not None:
			num = -self.finalArray[self.i][1]+self.coords.y
			den = self.finalArray[self.i][0]-self.coords.x
			self.displacement = np.sqrt(np.power(num,2)+np.power(den,2))
			self.angle2 = np.arctan2(num,den)
			self.del_theta = self.angle-self.angle2			#angle_difference
			
			
			
			
			if self.displacement<30:						#keeping shifting distance to 30 units
				self.zone = 2
			elif abs(self.del_theta)<1:						#keeping the angle error to '3' degrees
				self.zone = 1
			else:
				self.zone = 0
			# print('zone : '+ str(self.zone))

			if self.zone==0:
				if (self.del_theta>0 and self.del_theta<np.pi) or self.del_theta<-np.pi:
					self.action = 3							#go left
				elif (self.del_theta<0 and self.del_theta>-np.pi) or self.del_theta>np.pi:
					self.action = 2							#go right
			elif self.zone==1:
				self.action = 1								#go forward
			else:
				self.i +=1
				if self.i == len(self.finalArray):
					self.action = 4							#stop
					print('Hara Hara Maha Deva! The bot has completed the path and reached the destination!')
			print(str(self.displacement) +' | ' + str(self.i) + ' | '+str(self.angle)+' | ' +str(self.angle2)+' | '+ str(self.del_theta) +' | '+str(num)+' | '+str(den)+ ' | '+str(self.action))
			
			self.move_pub.publish(self.action)
			self.angle_kp_pub.publish(abs(self.del_theta))
			self.pos_kp_pub.publish(self.displacement/1000)

 		

if __name__=='__main__':
	a = applyAstar()
	r = rospy.Rate(30)

	while not rospy.is_shutdown():
		if a.finalArray is None:
			a.getPath()
		elif a.action!=4:
			a.align()
		r.sleep()
			


