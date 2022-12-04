#!/usr/bin/env python3

# trunc8 did this
import cv2
import rospy
import rospkg
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
from tracking.msg import circle
from tracking.msg import circles
from sensor_msgs.msg import Image

class Video_Publisher:
	def __init__(self,topic_name):
		rospy.init_node(topic_name+'_node')
		self.topic_name = topic_name
		#self.circle_pub = rospy.Publisher('/circle' circles, queue_size=50)
		self.pub = rospy.Publisher(self.topic_name, Image, queue_size=20)
		self.mask_pub = rospy.Publisher('mask_pub', Image, queue_size=10)
		self.rate = rospy.Rate(120)
		self.bridge = CvBridge()
		self.circleData = circle()
		self.circleArray = circles()

	def publishVideo(self, filename):
		video_capture = cv2.VideoCapture(filename)
		while not rospy.is_shutdown():
			while(video_capture.isOpened()):
				ret, frame = video_capture.read()
				if ret:
					try:
						hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
						mask = cv2.inRange(hsv, (10, 150, 150), (30, 255,255))
						imask = mask>0
						orange = np.zeros_like(frame, np.uint8)
						orange[imask] = frame[imask]
						ros_msg = self.bridge.cv2_to_imgmsg(hsv, "8UC3")
      
						# gray = cv2.cvtColor(hsv, cv2.COLOR_BGR2GRAY)
						# rows = gray.shape[0]
						# blur = cv2.GaussianBlur(gray, (5, 5), cv2.BORDER_DEFAULT)
						# ros_msg = self.bridge.cv2_to_imgmsg(hsv, "8UC3")
						# self.mask_pub.publish(ros_msg)
						# hough = cv2.HoughCircles(blur, cv2.HOUGH_GRADIENT, 1, rows / 8, param1=40, param2=30, minRadius=30, maxRadius=120)
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
						# 		cv2.circle(frame, (x, y), (int)(r), (0,255,255), 10)
						# 	self.circleArray = circles()
       
						ros_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
					except CvBridgeError as e:
						print(e)
						continue
					self.pub.publish(ros_msg)
					rospy.loginfo('publishing video')
					self.rate.sleep()
				else: #read in loop
					video_capture.set(cv2.CAP_PROP_POS_FRAMES, 0)
					continue
				
	def publishImage(self):
		rospack = rospkg.RosPack()
		filename = rospack.get_path("tracking") + "/image/ball_lab_pic.png"
		image = cv2.imread(filename, cv2.IMREAD_COLOR)
		while not rospy.is_shutdown():
			rospy.loginfo('publishing image')
			if image is not None:
					self.pub.publish(self.bridge.cv2_to_imgmsg(image))
			self.rate.sleep()
	

if __name__ == '__main__':
	try:
		pub_obj = Video_Publisher('video')
		# Not including RosPack causes error if this script is rosrun from a different directory
		rospack = rospkg.RosPack()
		filename = rospack.get_path("tracking") + "/video/ball_lab_video.mp4"
		pub_obj.publishVideo(filename)
		#pub_obj.publishImage()
	except rospy.ROSInterruptException:
		rospy.loginfo("Node terminated.")
