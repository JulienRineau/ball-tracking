#!/usr/bin/env python

import numpy as np
import cv2 as cv
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import imutils
from collections import deque
from std_msgs.msg import String


class BallDetector:
	def __init__(self,pub_topic="ball_coord"):
		self.buffer = 64
		self.pub_topic = pub_topic
		self.rate = rospy.Rate(120) #3Hz
		self.bridge = CvBridge()
		self.pts = deque(maxlen=self.buffer)
		
	def detection(self,topic,display=True):
		self.display = display
		self.str_pub = rospy.Publisher('string_pub', String, queue_size=10)
		self.img_pub = rospy.Publisher('circle_visualization', Image, queue_size=50)
		print("detection starting")
		rospy.Subscriber(topic, Image, self.trackerCallback)
		print("subscriber launched")
	
	def draw_circle(self, img, x, y, radius):
     	# loop over the set of tracked points
		for i in range(1, len(self.pts)):
			# if either of the tracked points are None, ignore
			# them
			if self.pts[i - 1] is None or self.pts[i] is None:
				continue
			# otherwise, compute the thickness of the line and
			# draw the connecting lines
			thickness = int(np.sqrt(self.buffer / float(i + 1)) * 2.5)
			cv.line(img, self.pts[i - 1], self.pts[i], (0, 0, 255), thickness)

		# draw the circle and center of the shape on the image
		cv.circle(img, (int(x), int(y)), int(radius),(0, 255, 255), 2)
		cv.circle(img, (int(x), int(y)), 7, (255, 255, 255), -1)
		cv.putText(img, "center", (x - 20, y - 20),
			cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

		img_msg = self.bridge.cv2_to_imgmsg(img, "8UC3")
		self.img_pub.publish(img_msg)
  
	def trackerCallback(self, ros_msg):
		try:
			img = self.bridge.imgmsg_to_cv2(ros_msg, "8UC3")
		except CvBridgeError as e:
			print(e)
			return

		#convert to hsv
		hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
		blur = cv.GaussianBlur(hsv, (7, 7), cv.BORDER_DEFAULT)
  
		# mask of orange 
		mask = cv.inRange(blur, (10, 150, 150), (20, 255,255))
		mask = cv.erode(mask, None, iterations=2)
		mask = cv.dilate(mask, None, iterations=2)
		orange = cv.bitwise_and(blur, blur, mask=mask)
  
		# find contours in the thresholded image		
		gray = cv.cvtColor(orange, cv.COLOR_BGR2GRAY)
		cnts = cv.findContours(gray.copy(), cv.RETR_EXTERNAL,
			cv.CHAIN_APPROX_SIMPLE)
		cnts = imutils.grab_contours(cnts)

		if cnts:
			c = max(cnts, key=cv.contourArea)
			((x, y), radius) = cv.minEnclosingCircle(c)
    
			x = round((x-404)/19.3,2)
			y = round((y-226)/18.8,2)
			radius = int(radius)
			self.str_pub.publish(str(x) + ' ' + str(y) + ' ' + str(radius))
   			
      		# update the points queue
			self.pts.appendleft((int(x),int(y)))
   
		if self.display:
			self.draw_circle(img, x, y, radius)
		
		self.img_pub.publish(self.bridge.cv2_to_imgmsg(img, "8UC3"))


if __name__ == '__main__':
	rospy.init_node('ball_detector_node')
	detector = BallDetector()
	detector.detection('/webcam/image_raw', display=True)
	rospy.spin()