#!/usr/bin/env python

# including libraries
import roslib
import sys
import rospy
import cv2
import math
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import matplotlib.pyplot as plt


MAP = np.array([[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],[0,1,1,1,0,1,1,1,1,1,0,1,1,1,1,1,0,1,1,0],[0,1,0,1,0,1,0,0,0,0,0,1,0,0,0,0,0,0,1,0],[0,1,0,1,1,1,0,1,1,1,1,1,0,1,0,1,1,1,1,0],[0,1,0,0,0,0,0,1,0,0,0,0,0,1,0,1,0,0,0,0],[0,1,1,1,1,1,0,1,1,1,0,1,0,1,1,1,1,1,1,0],[0,0,0,0,0,1,0,0,0,1,0,1,0,1,0,0,0,0,1,0],[0,1,1,1,0,1,0,1,1,1,0,1,1,1,0,1,1,1,1,0],[0,1,0,1,0,1,0,1,0,0,0,0,0,0,0,1,0,0,0,0],[0,1,0,1,0,1,0,1,0,1,1,1,0,1,1,1,1,1,1,0],[0,1,0,1,0,1,0,1,0,1,0,1,0,0,0,0,0,0,1,0],[0,1,0,1,1,1,0,1,0,1,0,1,1,1,0,1,1,1,1,0],[0,1,0,0,0,0,0,1,0,1,0,0,0,1,0,1,0,0,0,0],[0,1,1,1,1,1,0,1,1,1,0,1,0,1,1,1,1,1,1,0],[0,1,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0],[0,1,0,1,1,1,1,1,1,0,1,1,1,0,1,1,1,0,1,0],[0,1,0,1,0,0,0,0,1,0,1,0,1,0,1,0,1,0,1,0],[0,1,0,1,0,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0],[0,1,0,1,1,1,1,0,1,1,1,0,1,1,1,0,1,1,1,0],[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]])

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
		
		# crop out the labyrinth region (y by x)
		cv_image = cv_image[22:240, 44:268]
		# resize the image to 200x200 each region is 10x10
		cv_image = cv2.resize(cv_image, (400, 400)) 
		# transfer the image from RGB to HSV
		hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

		# Red Ball Segmentation
		lower_red = np.array([0,50,150])
		upper_red = np.array([50,150,250])
		temp_ball = cv2.inRange(hsv_image,lower_red,upper_red)
		# Erosion and Dilation processing
		kernel = np.ones((3,3),np.uint8)
		temp_ball = cv2.dilate(temp_ball,kernel,iterations = 2)
		#cv2.imshow("Red Ball", temp_ball)
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
		# Compensate with some noise
		radius = 10
		if abs(center[0]-position_history[0])+abs(center[1]-position_history[1])<=4:
			center = position_history
		cv2.circle(cv_image,center,radius,(0,255,0),2)
		position_history = center
		cv2.imshow("Ball tracking", cv_image)
		

		# manipulate the center coordinate to be the nearest 10 while extract the position in 20 by 20		
		# FIRST check who is more close to 0	
		checkx = center[0]%20-10
		checky = center[1]%20-15
		if abs(checkx) <= abs(checky):
			newx = center[0] - checkx
			newy = center[1]*0.955
		elif abs(checkx) > abs(checky):
			newx = center[0]
			newy = 0.955*(center[1] - checky) 
		newcenter = (newx, int(newy))
		# read the reference map for animation	
		map_ref = cv2.imread('/home/sunyue/catkin_ws/src/tracking/map.png')
		cv2.circle(map_ref,newcenter,radius,(0,0,255),-5)
		
		# SECOND transfer the real location to the 20x20 grid
		gridx = newcenter[0]/20+1
		gridy = newcenter[1]/20+1



		# A* for path planning
		goal = [10,2]
		current = [gridx, gridy]
		precheck = abs(current[0]-goal[0])+abs(current[1]-goal[1])
		if precheck == 0: check = 0
		else: check = 100
		path = np.array([current])
		backup = np.array([[0,0,0,0]])


		while check!=0:
			# generate the potential candidate
			north = [current[0],current[1]-1]
			south = [current[0],current[1]+1]
			east = [current[0]+1,current[1]]
			west = [current[0]-1,current[1]]

			#print current

			# calculate the heuristic
			n_heuristic = math.sqrt(pow(north[0]-goal[0],2)+pow(north[1]-goal[1],2))
			s_heuristic = math.sqrt(pow(south[0]-goal[0],2)+pow(south[1]-goal[1],2))
			e_heuristic = math.sqrt(pow(east[0]-goal[0],2)+pow(east[1]-goal[1],2))
			w_heuristic = math.sqrt(pow(west[0]-goal[0],2)+pow(west[1]-goal[1],2))

			# check the punishment of obstacle
			if MAP[north[1]-1,north[0]-1]==0: n_punish = 2000
			else: n_punish = 0
			if MAP[south[1]-1,south[0]-1]==0: s_punish = 2000
			else: s_punish = 0
			if MAP[east[1]-1,east[0]-1]==0: e_punish = 2000
			else: e_punish = 0
			if MAP[west[1]-1,west[0]-1]==0: w_punish = 2000
			else: w_punish = 0

			#print n_punish, s_punish, e_punish, w_punish
			# check last node never go back
			num = path.shape[0] # get the path step number
			if num!=1:
				last_step = path[-2]
				n_check = north - last_step
				s_check = south - last_step
				e_check = east - last_step
				w_check = west - last_step
				if ( n_check[0]==0 and n_check[1]==0):  n_punish = 2000
				if ( s_check[0]==0 and s_check[1]==0):  s_punish = 2000
				if ( e_check[0]==0 and e_check[1]==0):  e_punish = 2000
				if ( w_check[0]==0 and w_check[1]==0):  w_punish = 2000

			# sum the cost together
			n_cost = int(n_heuristic + n_punish)
			s_cost = int(s_heuristic + s_punish)
			e_cost = int(e_heuristic + e_punish)
			w_cost = int(w_heuristic + w_punish)
			cost = [n_cost, s_cost, e_cost, w_cost]


			# there will be some situations should be taken into consideration
			index = np.argmin(cost) # where the smallest cost is located
			mincost = cost[index]
			# First only one direction cost is less than 1000, then just pick that
			if mincost<=1000: # there must be at least one solution
				sumcheck = cost[0]+cost[1]+cost[2]+cost[3]
				if sumcheck >= 6000: # only one next choice
					if index == 0: next = north
					elif index == 1: next = south
					elif index == 2: next = east
					elif index == 3: next = west
					# update the path
					path = np.append(path,[next],axis=0)
					# update the check for next while
					precheck = abs(next[0]-goal[0])+abs(next[1]-goal[1])
					if precheck == 0:
						check = 0
					# updat the current
					current = next

				elif (sumcheck >= 4000 and sumcheck < 6000) : # two posible choices
					if index == 0: next = north
					elif index == 1: next = south
					elif index == 2: next = east
					elif index == 3: next = west
					# update the path choose the one have the least cost
					path = np.append(path,[next],axis=0)
					# update the check for next while
					precheck = abs(next[0]-goal[0])+abs(next[1]-goal[1])
					if precheck == 0:
						check = 0
					# save the branch to the back up [current, branch]
					fakecost = cost
					fakecost[index] = 2000	# mannually fake the minimum cost choice
					fakeindex = np.argmin(fakecost) # where the smallest cost is located
					if fakeindex == 0: branch = north
					elif fakeindex == 1: branch = south
					elif fakeindex == 2: branch = east
					elif fakeindex == 3: branch = west
					backup = np.append([[current[0],current[1],branch[0],branch[1]]], backup, axis=0)
					# updat the current
					current = next

				elif (sumcheck >= 2000 and sumcheck < 4000) : # three posible choices
					if index == 0: next = north
					elif index == 1: next = south
					elif index == 2: next = east
					elif index == 3: next = west
					# update the path choose the one have the least cost
					path = np.append(path,[next],axis=0)
					# update the check for next while
					precheck = abs(next[0]-goal[0])+abs(next[1]-goal[1])
					if precheck == 0:
						check = 0
					# save the branch to the back up [current, branch]
					# second cost
					secondcost = cost
					secondcost[index] = 2000	# mannually fake the minimum cost choice
					secondindex = np.argmin(secondcost) # where the smallest cost is located
					if secondindex == 0: branch1 = north
					elif secondindex == 1: branch1 = south
					elif secondindex == 2: branch1 = east
					elif secondindex == 3: branch1 = west

					thirdcost = secondcost
					thirdcost[secondindex] = 2000	# mannually fake the minimum cost choice
					thirdindex = np.argmin(thirdcost) # where the smallest cost is located
					if thirdindex == 0: branch2 = north
					elif thirdindex == 1: branch2 = south
					elif thirdindex == 2: branch2 = east
					elif thirdindex == 3: branch2 = west
					# update branch based on cost difference
					backup = np.append([[current[0],current[1],branch2[0],branch2[1]]], backup, axis=0)
					backup = np.append([[current[0],current[1],branch1[0],branch1[1]]], backup, axis=0)
					# updat the current
					current = next



			elif mincost>=2000: # there is no next choice we have go to backup branchs
				# next step is the first ranking branch				
				next = [backup[0,2],backup[0,3]]
				# cut the path back
				current = [backup[0,0],backup[0,1]]
				compare = abs(path-current)
				summation = sum(np.transpose(compare))
				index = np.argmin(summation)
				# cut the path from 0 to current one
				path = path[:index+1]
				# update the path with next step
				path = np.append(path,[next],axis=0)
				# delete the first backup
				backup = backup[1:]
				# update the check for next while
				precheck = abs(next[0]-goal[0])+abs(next[1]-goal[1])
				if precheck == 0:
					check = 0
				# updat the current
				current = next
		
		# A* algorithm is ended

		steps = path.shape[0]
		i = 0
		while i < steps-1:
			cv2.line(map_ref,(20*path[i,0]-10,20*path[i,1]-10),(20*path[i+1,0]-10,20*path[i+1,1]-10),(255,0,0),3)
			i = i+1

		cv2.imshow("Map Image", map_ref)

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
