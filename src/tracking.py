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

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("final_image",Image)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
    except CvBridgeError, e:
      print e

    #enter CV code
    #hsv_image=cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    #cv_unedited=cv_image
    #lower_red = np.array([0,124,66])
    #upper_red = np.array([91,255,245])
    #thres=cv2.inRange(hsv_image,lower_red,upper_red)

    #kernel = np.ones((5,5),np.uint8)
    #erosion = cv2.erode(thres,kernel,iterations = 2)
    #dilation = cv2.dilate(erosion,kernel,iterations = 2)

    #contours,hierarcy = cv2.findContours(dilation,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

    #cv2.namedWindow=("Original Image", 1)
    cv2.imshow("Original Image", cv_image)
    #(width, height) = cv_image.size
    #cv2.namedWindow=("HSV Image", 2)
    #cv2.imshow("HSV Image", hsv_image)
    #cv2.namedWindow=("Thresholded Image", 3)
    #cv2.imshow("Thresholded Image", thres)
    #cv2.namedWindow=("Morphed Image", 4)
    #cv2.imshow("Morphed Image", dilation)
    #max_area = 0
    #best_cnt = 0
    #for cnt in contours:
    #  area=cv2.contourArea(cnt)
    #  if area > max_area:
    #    max_area=area
    #    best_cnt=cnt
    #cv2.drawContours(cv_image, contours, -1, (0,255,0), 3)
    #  (x,y),radius = cv2.minEnclosingCircle(best_cnt)
    #  center=(int(x),int(y))
    #  radius=int(radius)
    #  cv2.circle(cv_image,center,radius,(0,255,0),2)
    #cv2.namedWindow=("Tracking", 5)
    #cv2.imshow("Tracked Image",cv_image)
    cv2.waitKey(1)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8"))
    except CvBridgeError, e:
      print e

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv2.destroyAllWindows()
 
if __name__ == '__main__':
    main(sys.argv)
