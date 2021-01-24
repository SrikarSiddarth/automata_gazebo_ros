#!/usr/bin/env python

import numpy as np
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import rospy
from geometry_msgs.msg import Point32
from std_msgs.msg import Float32

#assuming that the triangle is always red and rectangle is always blue
class tracer():
	"""tracer class"""
	def __init__(self):
		rospy.init_node('tracer')
		self.angle = Float32()
		self.bridge = CvBridge()
		self.position = Point32()
		self.pos_pub = rospy.Publisher("/position_info",Point32,queue_size=10)
		self.angle_pub = rospy.Publisher("/direction_info",Float32,queue_size=10)
		self.image_sub = rospy.Subscriber("/gazebo/overhead_cam/image",Image,self.image_callback)
		self.res=None
		self.blue_detected = False
		self.red_detected = False

	def image_callback(self, data):
		try:
			rec = self.bridge.imgmsg_to_cv2(data, "bgr8")
			# print(rec.shape)
			#slicing the image
			
			
			rec2 = rec[20:1275, 30:1215]				#this is for arena.jpg

			height = 720
			width = 720
			dim = (height, width)
			self.res = cv.resize(rec2, dim, interpolation = cv.INTER_LINEAR)
			# cv.imshow('d', self.res)
			# cv.waitKey(0)
			
			
		except CvBridgeError as e:
 			print(e)

 	def track(self):
 		if self.res is not None:
			hsv = cv.cvtColor(self.res, cv.COLOR_BGR2HSV)
			lower_blue = np.array([90,50,50])
			im = np.copy(hsv)
			 # define range of blue color in HSV
			upper_blue = np.array([130,255,255])
			                    # Threshold the HSV image to get only blue colors
			maskBlue = cv.inRange(hsv, lower_blue, upper_blue)
			
			_, cntBlue ,hierarchy = cv.findContours(maskBlue , cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE )



			# #this code is used to draw the contours present in cntBlue
			for i in range(len(cntBlue)):
				# print(cv.contourArea(cntBlue[i]))
				# cv.drawContours(im, [cntBlue[i]], 0, (0,0,0), 3)
				# cv.imshow('t', im)
				# cv.waitKey(0)
				if cv.contourArea(cntBlue[i])>400:
					Mb = cv.moments(cntBlue[i])
					self.blue_detected = True
				
				# im = np.copy(hsv)




			#after experimentation the second contour is the right contour

			if self.blue_detected:
				xb = (Mb['m10']/Mb['m00'])
				yb = (Mb['m01']/Mb['m00'])




			                    # define range of red color in HSV
			lower_red1 = np.array([160,100, 100])
			upper_red1 = np.array([179 ,255,255])
			maskRed1 = cv.inRange(hsv, lower_red1, upper_red1)
			lower_red2 = np.array([0, 100, 100])
			upper_red2 = np.array([10 ,255,255])
			maskRed2 = cv.inRange(hsv, lower_red2, upper_red2)
			                    # Threshold the HSV image to get only red colors

			maskRed = cv.addWeighted(maskRed1, 1, maskRed2, 1, 0)

			_, cntRed ,hierarchy = cv.findContours(maskRed , 1, cv.CHAIN_APPROX_SIMPLE )

			for i in range(len(cntRed)):
				# print('contour Area : ' + str(cv.contourArea(cntRed[i])))
				# cv.drawContours(im, [cntBlue[i]], 0, (0,0,0), 3)
				# cv.imshow('t', im)
				# cv.waitKey(0)
				if cv.contourArea(cntRed[i])>150:
					Mr = cv.moments(cntRed[i])
					self.red_detected = True
			
			if self.red_detected:
				xr = (Mr['m10']/Mr['m00'])
				yr = (Mr['m01']/Mr['m00'])

			if self.blue_detected and self.red_detected:
				self.angle = np.arctan2((yb-yr),(xr-xb))
					
				# cv.circle(self.res,(int(xr) , int(yr)), 3, (0,0,0), -1)
				# cv.circle(self.res,(int(xb) , int(yb)), 3, (0,0,0), -1)
				self.position.x, self.position.y = ( int((xr+xb)*0.5) , int((yr+yb)*0.5) )
				self.res = cv.putText(self.res, str((self.position.x, self.position.y)), (self.position.x, self.position.y), cv.FONT_HERSHEY_SIMPLEX , 0.4, (255,100,100), 1, cv.LINE_AA) 
				cv.imshow('g', self.res)
				cv.waitKey(33)

				# print('XR-XB : ' + str(xr-xb))
				# print('YR-YB : ' + str(yr-yb))


				# print('angle : ' + str(self.angle))
				# print('taninv : ' + str((np.arctan((yr-yb)/(xr-xb)))/np.pi*180))

				# print('location : ' + str(self.position))
				self.pos_pub.publish(self.position)
				self.angle_pub.publish(self.angle)

if __name__ == '__main__':
	t = tracer()
	r = rospy.Rate(30)
	while not rospy.is_shutdown():
		t.track()
		r.sleep()

    
