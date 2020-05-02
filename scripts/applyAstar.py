#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import cv2
import numpy as np
import Astar

class applyAstar():
	"""docstring for applyAstar"""
	def __init__(self):
		rospy.init_node('applyAstar')
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/gazebo/overhead_cam/image_rect_color",Image,self.image_callback)
		self.img=None
		self.res=None
		self.flag = 0
		self.flag2 = 0
		self.finalArray = None

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

 	def run(self):
 		if self.img is not None:
 			start_coords = [14, 14]
 			end_coords = [5, 0]
 			self.finalArray = Astar.play(self.img, start_coords, end_coords)
 			self.flag2 = 1
	 		for i in range(len(self.finalArray)):
	 			cv2.circle(self.img, (int(self.finalArray[i][0]), int(self.finalArray[i][1])), 3, 125, -1)
			cv2.imshow('d', self.img)
			cv2.waitKey(0)	
			np.save('/home/phani/catkin_ws/src/automata/scripts/finalArray', self.finalArray)
			# print(self.finalArray)
			print('finalArray saved.')
			exit()

 		

if __name__=='__main__':
	a = applyAstar()
	r = rospy.Rate(30)
	while a.flag2==0 and not rospy.is_shutdown():
		a.run()
		r.sleep()
			


