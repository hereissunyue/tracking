#!/usr/bin/env python

# including libraries
import roslib
#roslib.load_manifest('tracking')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import matplotlib.pyplot as plt


MAP = np.array([[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],[0,1,1,1,0,1,1,1,1,1,0,1,1,1,1,1,0,1,1,0],[0,1,0,1,0,1,0,0,0,0,0,1,0,0,0,0,0,0,1,0],[0,1,0,1,1,1,0,1,1,1,1,1,0,1,0,1,1,1,1,0],[0,1,0,0,0,0,0,1,0,0,0,0,0,1,0,1,0,0,0,0],[0,1,1,1,1,1,0,1,1,1,0,1,0,1,1,1,1,1,1,0],[0,0,0,0,0,1,0,0,0,1,0,1,0,1,0,0,0,0,1,0],[0,1,1,1,0,1,0,1,1,1,0,1,1,1,0,1,1,1,1,0],[0,1,0,1,0,1,0,1,0,0,0,0,0,0,0,1,0,0,0,0],[0,1,0,1,0,1,0,1,0,1,1,1,0,1,1,1,1,1,1,0],[0,1,0,1,0,1,0,1,0,1,0,1,0,0,0,0,0,0,1,0],[0,1,0,1,1,1,0,1,0,1,0,1,1,1,0,1,1,1,1,0],[0,1,0,0,0,0,0,1,0,1,0,0,0,1,0,1,0,0,0,0],[0,1,1,1,1,1,0,1,1,1,0,1,0,1,1,1,1,1,1,0],[0,1,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0],[0,1,0,1,1,1,1,1,1,0,1,1,1,0,1,1,1,0,1,0],[0,1,0,1,0,0,0,0,1,0,1,0,1,0,1,0,1,0,1,0],[0,1,0,1,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0],[0,1,0,1,1,1,1,0,1,1,1,0,1,1,1,0,1,1,1,0],[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]])

position_history = (0,0)


class labyrinth_solver:

	def __init__(self):
		self.image_pub = rospy.Publisher("final_image",Image)
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)

	def callback(self,data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
		except CvBridgeError, e:
			print e
		
		# crop out the labyrinth region
		cv_image = cv_image[16:243, 40:268]
		# resize the image to 200x200 each region is 10x10
		cv_image = cv2.resize(cv_image, (200, 200)) 
		# transfer the image from RGB to HSV
		hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

		# Red Ball Segmentation
		lower_red = np.array([0,100,180])
		upper_red = np.array([50,150,250])
		temp_ball = cv2.inRange(hsv_image,lower_red,upper_red)
		# Erosion and Dilation processing
		kernel = np.ones((5,5),np.uint8)
		temp_ball = cv2.dilate(temp_ball,kernel,iterations = 1)
		cv2.imshow("Red Ball", temp_ball)
		# Calculate the contour
		contours,hierarcy = cv2.findContours(temp_ball,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
		# Select the biggest contout as the target		
		max_area = 0
		for cnt in contours:
			area=cv2.contourArea(cnt)
			if area > max_area:
				max_area=area
				target = cnt

		global position_history # calling global variable
		# handling with target missing
		if max_area >= 10:
			(x,y),radius = cv2.minEnclosingCircle(target)
			center = (int(x),int(y))
		else:
			center = position_history

		print center
		radius = 5
		cv2.circle(cv_image,center,radius,(0,255,0),2)	
		position_history = center

		#cv2.imshow("Red Ball", temp_ball)
		cv2.imshow("Original Image", cv_image)
		cv2.waitKey(1)

		try:
			self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8"))
		except CvBridgeError, e:
			print e

def main(args):
	ic = labyrinth_solver()
	rospy.init_node('labyrinth_solver', anonymous=True)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print "Shutting down"
	cv2.destroyAllWindows()
 
if __name__ == '__main__':
		main(sys.argv)
