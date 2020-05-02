import numpy as np

def angle_three_sixty(yr-yb, xr-xb):
	# d = ((xr-xb)**2 + (yr-yb)**2)**0.5
	if xr-xb == 0 and yr-yb<0:										#positive yaxis
		angle = 90
		# print('positive y axis')
	elif xr-xb == 0 and yr-yb>0:									#negative y axis
		angle = 270
		# print('negative y axis')
	elif yr-yb ==0 and xr-xb<0:
		angle = 180
		# print('negative x axis')
	elif yr-yb == 0 and xr-xb>0:
		angle = 0
		# print('positive x axis')
	elif xr-xb>0 and yr-yb>0 :										#fourth quadrant
		angle =(np.pi*2-np.arctan((yr-yb)/(xr-xb)))/np.pi*180
		# print('4th quadrant')
	elif (xr-xb<0 and yr-yb>0) or (xr-xb<0 and yr-yb<0):
		angle =(np.pi-np.arctan((yr-yb)/(xr-xb)))/np.pi*180 
		# print('2nd or 3rd quadrant')
	else:
		# print('1st quadrant')	
		angle = -(np.arctan((yr-yb)/(xr-xb)))/np.pi*180

	return angle