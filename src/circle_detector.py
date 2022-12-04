#!/usr/bin/env python3

import numpy as np
import cv2 as cv
import rospkg
import rospy
from tracking.msg import circle
from tracking.msg import circles
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import imutils

class Circle_Detector:
	def __init__(self,pub_topic="circle",radMin=30,radMax=120):
		self.pub_topic = pub_topic
		self.radMin = radMin
		self.radMax = radMax
		self.rate = rospy.Rate(120) #3Hz
		self.circleData = circle()
		self.circleArray = circles()
		self.bridge = CvBridge()

	def detection(self,topic,display=True):
		#rospy.init_node(self.pub_topic+'_node', anonymous=True)
		self.display = display
		self.circle_pub = rospy.Publisher(self.pub_topic, circles, queue_size=50)
		self.img_pub = rospy.Publisher('circle_visualization', Image, queue_size=50)
		self.subscription_pub = rospy.Publisher('subscription', Image, queue_size=50)
		self.mask_pub = rospy.Publisher('mask', Image, queue_size=50)
		print("detection starting")
		rospy.Subscriber(topic, Image, self.trackerCallback)
		print("subscriber launched")
	
	def draw_circle(self, img, list):
		for cx, cy, radius in list:
			#draw circle
			cv.circle(img, (cx, cy), (int)(radius), (0,255,255), 10)
		ros_msg = self.bridge.cv2_to_imgmsg(img, "8UC3")
		self.img_pub.publish(ros_msg)
  

  
	def trackerCallback(self, ros_msg):
		"""Publish the position and size of the detected circles

		Parameters
		----------
		first : ros_msg (sensor_msgs.msg Image)
			Ros image from the video
		"""
		#print(ros_msg)
		try:
			img = self.bridge.imgmsg_to_cv2(ros_msg, "8UC3")
		except CvBridgeError as e:
			print(e)
			return

		#convert to hsv
		hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

		blur = cv.GaussianBlur(hsv, (5, 5), cv.BORDER_DEFAULT)
  
		## mask of orange 
		mask = cv.inRange(blur, (10, 150, 150), (20, 255,255))
		orange = cv.bitwise_and(blur, blur, mask=mask)
		## slice the orange
		# imask = mask>0
		# orange = np.zeros_like(img, np.uint8)
		# orange[imask] = img[imask]
  
		gray = cv.cvtColor(orange, cv.COLOR_BGR2GRAY)
  
		
  
		# find contours in the thresholded image
		cnts = cv.findContours(gray.copy(), cv.RETR_EXTERNAL,
			cv.CHAIN_APPROX_SIMPLE)

		cnts = imutils.grab_contours(cnts)
		# loop over the contours
		for c in cnts:
			# compute the center of the contour
			M = cv.moments(c)
			if M["m00"] != 0:
				cX = int(M["m10"] / M["m00"])
				cY = int(M["m01"] / M["m00"])
			else:
				# set values as what you need in the situation
				cX, cY = 0, 0
    
			# draw the contour and center of the shape on the image
			cv.drawContours(img, [c], -1, (0, 255, 0), 2)
			cv.circle(img, (cX, cY), 7, (255, 255, 255), -1)
			cv.putText(img, "center", (cX - 20, cY - 20),
				cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
		mask_msg = self.bridge.cv2_to_imgmsg(orange, "8UC3")
		circle_msg = self.bridge.cv2_to_imgmsg(img, "8UC3")
		self.mask_pub.publish(mask_msg)
		self.img_pub.publish(circle_msg)
		print('publishing')

		#convert to grayscale
		# gray = cv.cvtColor(orange, cv.COLOR_BGR2GRAY)
		# rows = gray.shape[0]
  
		# #blur to remove noise
		# blur = cv.GaussianBlur(gray, (5, 5), cv.BORDER_DEFAULT)
		
  		# #circle detection
		# hough = cv.HoughCircles(blur, cv.HOUGH_GRADIENT, 1, rows / 8, param1=40, param2=30, minRadius=30, maxRadius=120)

		# ensure at least some circles were found
		# circle_to_draw = []
		# if hough is not None:
		# 	print("Hough detection")
		# 	# convert the (x, y) coordinates and radius of the circles to integers
		# 	hough = np.round(hough[0, :]).astype("int")
		# 	for x, y, r in hough:
		# 		self.circleData.x = x
		# 		self.circleData.y = y
		# 		self.circleData.radius = r
		# 		self.circleArray.circles.append(self.circleData)
		# 		circle_to_draw.append([x,y,r])
		# 	self.circle_pub.publish(self.circleArray)
		# 	self.circleArray = circles()
		# 	#self.rate.sleep()
		# if self.display:
		# 	self.draw_circle(img,circle_to_draw)

if __name__ == '__main__':
	rospy.init_node('number_counter')
	detector = Circle_Detector('ball_coord')
	detector.detection('/webcam/image_raw', display=True)
	rospy.spin()